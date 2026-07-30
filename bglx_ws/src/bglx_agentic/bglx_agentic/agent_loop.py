"""Prompt in, robot behaviour out.

    ros2 run bglx_agentic agent

Backends, selected with BGLX_BACKEND:
    ollama     (default) local model, no key, no network egress
    openai     any OpenAI-compatible endpoint: DeepSeek, Kimi, Qwen, GLM
    anthropic  Claude API, needs ANTHROPIC_API_KEY and a route to
               api.anthropic.com

Everything below this file -- tool dispatch, ROS, the failure translator --
is backend-agnostic. Only complete() and the schema conversion differ.
"""

import json
import os
import sys
import time

import rclpy
import requests

from .robot_tools import RobotTools, _spin
from .tool_schemas import TOOLS, SYSTEM_PROMPT
from .trace_log import TraceLog

BACKEND = os.environ.get("BGLX_BACKEND", "ollama")
OLLAMA_HOST = os.environ.get("OLLAMA_HOST", "http://127.0.0.1:11434")
OLLAMA_MODEL = os.environ.get("BGLX_MODEL", "qwen2.5:7b")
ANTHROPIC_MODEL = os.environ.get("BGLX_MODEL", "claude-sonnet-4-6")

# Any OpenAI-compatible endpoint: DeepSeek, Moonshot/Kimi, Qwen/DashScope,
# Zhipu/GLM, vLLM, LM Studio. All reachable from mainland China.
OPENAI_BASE = os.environ.get("BGLX_BASE_URL", "https://api.deepseek.com/v1")
OPENAI_MODEL = os.environ.get("BGLX_MODEL", "deepseek-chat")
OPENAI_KEY_VAR = os.environ.get("BGLX_KEY_VAR", "BGLX_API_KEY")

MAX_STEPS = 40
MAX_TOKENS = 2000
MAX_NUDGES = 2      # retries when the model describes a call instead of making one


def to_openai_tools(tools):
    """Anthropic tool schema -> OpenAI/Ollama function schema."""
    out = []
    for t in tools:
        params = dict(t["input_schema"])
        params.setdefault("properties", {})
        out.append({
            "type": "function",
            "function": {
                "name": t["name"],
                "description": t["description"],
                "parameters": params,
            },
        })
    return out


class ToolCall:
    """Backend-neutral tool call."""

    def __init__(self, name, args, call_id=None):
        self.name = name
        self.args = args or {}
        self.id = call_id


class OllamaBackend:
    name = "ollama"

    def __init__(self):
        self.tools = to_openai_tools(TOOLS)
        try:
            r = requests.get(OLLAMA_HOST + "/api/tags", timeout=5)
            r.raise_for_status()
        except Exception as exc:
            sys.exit("Cannot reach Ollama at %s (%s). Is 'ollama serve' "
                     "running?" % (OLLAMA_HOST, exc))
        names = [m["name"] for m in r.json().get("models", [])]
        if OLLAMA_MODEL not in names:
            print("[warn] model '%s' not pulled. Available: %s"
                  % (OLLAMA_MODEL, ", ".join(names) or "none"))

    def start(self, task):
        return [{"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": task}]

    def complete(self, history):
        r = requests.post(
            OLLAMA_HOST + "/api/chat",
            json={"model": OLLAMA_MODEL, "messages": history,
                  "tools": self.tools, "stream": False,
                  "options": {"temperature": 0.2,
                              "num_predict": MAX_TOKENS}},
            timeout=300,
        )
        r.raise_for_status()
        msg = r.json()["message"]
        history.append(msg)

        text = (msg.get("content") or "").strip()
        calls = []
        for c in msg.get("tool_calls") or []:
            fn = c.get("function", {})
            args = fn.get("arguments")
            if isinstance(args, str):
                try:
                    args = json.loads(args)
                except json.JSONDecodeError:
                    args = {}
            calls.append(ToolCall(fn.get("name"), args))
        return text, calls

    def add_results(self, history, results):
        for call, out in results:
            history.append({"role": "tool", "name": call.name,
                            "content": str(out)})


class OpenAICompatBackend:
    """DeepSeek, Kimi, Qwen, GLM, vLLM -- anything speaking the OpenAI API."""

    name = "openai-compatible"

    def __init__(self):
        self.tools = to_openai_tools(TOOLS)
        self.key = os.environ.get(OPENAI_KEY_VAR)
        if not self.key:
            sys.exit("%s is not set. Export your provider API key, e.g.\n"
                     "  export %s=sk-..." % (OPENAI_KEY_VAR, OPENAI_KEY_VAR))

    def start(self, task):
        return [{"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": task}]

    def complete(self, history):
        r = requests.post(
            OPENAI_BASE.rstrip("/") + "/chat/completions",
            headers={"Authorization": "Bearer " + self.key,
                     "Content-Type": "application/json"},
            json={"model": OPENAI_MODEL, "messages": history,
                  "tools": self.tools, "temperature": 0.2,
                  "max_tokens": MAX_TOKENS},
            timeout=300,
        )
        if r.status_code != 200:
            raise RuntimeError("HTTP %d: %s" % (r.status_code, r.text[:300]))
        msg = r.json()["choices"][0]["message"]
        history.append(msg)

        text = (msg.get("content") or "").strip()
        calls = []
        for c in msg.get("tool_calls") or []:
            fn = c.get("function", {})
            args = fn.get("arguments")
            if isinstance(args, str):
                try:
                    args = json.loads(args)
                except json.JSONDecodeError:
                    args = {}
            calls.append(ToolCall(fn.get("name"), args, c.get("id")))
        return text, calls

    def add_results(self, history, results):
        for call, out in results:
            history.append({"role": "tool", "tool_call_id": call.id,
                            "content": str(out)})


class AnthropicBackend:
    name = "anthropic"

    def __init__(self):
        if not os.environ.get("ANTHROPIC_API_KEY"):
            sys.exit("ANTHROPIC_API_KEY is not set.")
        try:
            from anthropic import Anthropic
        except ImportError:
            sys.exit("pip install anthropic")
        self.client = Anthropic()

    def start(self, task):
        return [{"role": "user", "content": task}]

    def complete(self, history):
        resp = self.client.messages.create(
            model=ANTHROPIC_MODEL, max_tokens=MAX_TOKENS,
            system=SYSTEM_PROMPT, tools=TOOLS, messages=history)
        history.append({"role": "assistant", "content": resp.content})
        text = " ".join(b.text.strip() for b in resp.content
                        if b.type == "text" and b.text.strip())
        calls = [ToolCall(b.name, b.input, b.id)
                 for b in resp.content if b.type == "tool_use"]
        return text, calls

    def add_results(self, history, results):
        history.append({"role": "user", "content": [
            {"type": "tool_result", "tool_use_id": call.id,
             "content": str(out)} for call, out in results]})


class Agent:
    def __init__(self, tools, backend, trace=None):
        self.tools = tools
        self.backend = backend
        self.trace = trace

    def _dispatch(self, name, args):
        table = {
            "get_pose": lambda: self.tools.get_pose(),
            "get_scan_summary": lambda: self.tools.get_scan_summary(),
            "check_attitude": lambda: self.tools.check_attitude()[1],
            "list_landmarks": lambda: self.tools.list_landmarks(),
            "navigate_to": lambda: self.tools.navigate_to(
                args["x"], args["y"], args.get("yaw")),
            "navigate_relative": lambda: self.tools.navigate_relative(
                args.get("forward", 0.0), args.get("left", 0.0)),
            "turn_by": lambda: self.tools.turn_by(args["degrees"]),
            "mark_here": lambda: self.tools.mark_here(args["name"]),
            "navigate_to_landmark": lambda: self.tools.navigate_to_landmark(
                args["name"]),
            "drive": lambda: self.tools.drive(
                args["linear_x"], args["angular_z"], args["duration"]),
            "wait": lambda: self.tools.wait(args["seconds"]),
            "get_last_failure": lambda: self.tools.get_last_failure(),
            "stop": lambda: self.tools.stop(),
        }
        fn = table.get(name)
        if fn is None:
            return "Unknown tool: %s. Valid tools: %s" % (
                name, ", ".join(sorted(table)))
        try:
            return fn()
        except KeyError as exc:
            return "Tool '%s' missing required argument %s" % (name, exc)
        except Exception as exc:
            return "Tool '%s' raised %s: %s" % (name, type(exc).__name__, exc)

    def _describes_a_tool(self, text):
        """Did the model talk about calling a tool instead of calling it?

        Small models frequently emit 'drive(-0.5, 0, 2)' as prose. The tool
        call never happens, the robot never moves, and without this check the
        harness reports task completion.
        """
        if not text:
            return False
        names = ("get_pose", "get_scan_summary", "navigate_to", "drive",
                 "wait", "stop", "list_landmarks", "get_last_failure",
                 "navigate_relative", "turn_by", "mark_here", "check_attitude",
                 "navigate_to_landmark")
        return any(n in text for n in names)

    def run(self, task):
        if self.trace:
            self.trace.task_start(task)
        history = self.backend.start(task)
        nudges = 0
        acted = False      # has any tool run this task?
        failed_calls = set()   # (tool, args) that already produced a failure

        for _ in range(MAX_STEPS):
            try:
                text, calls = self.backend.complete(history)
            except Exception as exc:
                print("\n[error] model call failed: %s" % exc)
                self.tools.stop()
                return

            if text:
                print("\n[agent] %s" % text)
                if self.trace:
                    self.trace.agent_text(text)

            if not calls:
                # Only nudge if NOTHING has executed yet. Once a tool has run,
                # a text-only reply is the model finishing the task, and
                # nudging it drives the robot again without being asked.
                if (not acted and self._describes_a_tool(text)
                        and nudges < MAX_NUDGES):
                    nudges += 1
                    print("[harness] no tool call emitted - nudging (%d/%d)"
                          % (nudges, MAX_NUDGES))
                    if self.trace:
                        self.trace.nudge(nudges)
                    history.append({
                        "role": "user",
                        "content": ("You described a tool call in text but did "
                                    "not actually emit one, so nothing "
                                    "happened and the robot did not move. "
                                    "Emit a real tool call now."),
                    })
                    continue
                if self.trace:
                    self.trace.task_end("complete", self.trace.step)
                return

            acted = True
            results = []
            for call in calls:
                print("[tool ] %s(%s)" % (call.name, call.args))
                sig = (call.name, json.dumps(call.args, sort_keys=True))
                if sig in failed_calls:
                    out = ("BLOCKED BY HARNESS: this exact call already failed "
                           "in this task and was not retried. Read the earlier "
                           "DIAGNOSIS and choose different arguments or a "
                           "different tool.")
                    print("[obs  ] %s\n" % out)
                    results.append((call, out))
                    continue

                pose_before = self.tools._pose()
                t_call = time.time()
                out = self._dispatch(call.name, call.args)
                elapsed = time.time() - t_call
                pose_after = self.tools._pose()
                upright, _att = self.tools.check_attitude()
                if upright is False:
                    out = str(out) + " " + _att
                print("[obs  ] %s\n" % out)
                _o = str(out)
                if ("DIAGNOSIS:" in _o
                        or _o.startswith(("REJECTED", "BLOCKED"))
                        or "refused" in _o[:60]
                        or "missing required argument" in _o
                        or _o.startswith("Unknown tool")
                        or " raised " in _o[:60]):
                    failed_calls.add(sig)
                if self.trace:
                    self.trace.tool_call(call.name, call.args, out, elapsed,
                                         pose_before, pose_after)
                results.append((call, out))
            self.backend.add_results(history, results)

        print("\n[agent] Step limit reached. Stopping.")
        if self.trace:
            self.trace.task_end("step_limit", self.trace.step)
        self.tools.stop()


def main(argv=None):
    backends = {"ollama": OllamaBackend,
                "openai": OpenAICompatBackend,
                "anthropic": AnthropicBackend}
    if BACKEND not in backends:
        sys.exit("Unknown BGLX_BACKEND '%s'. Choose from: %s"
                 % (BACKEND, ", ".join(backends)))
    backend = backends[BACKEND]()

    rclpy.init(args=argv)
    tools = RobotTools()
    _spin(tools)
    time.sleep(2.0)

    print("\n=== HEALTH ===")
    print(tools.health())
    model = {"ollama": OLLAMA_MODEL, "openai": OPENAI_MODEL,
             "anthropic": ANTHROPIC_MODEL}[BACKEND]
    print("\nBackend: %s (%s)" % (backend.name, model))
    print("Describe a task, 'quit' to exit.\n")

    trace = TraceLog(BACKEND, model)
    print("Trace: %s" % trace.path)
    agent = Agent(tools, backend, trace)
    try:
        while True:
            task = input("task> ").strip()
            if not task:
                continue
            if task in ("quit", "exit"):
                break
            agent.run(task)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        try:
            tools.stop()
        except Exception:
            pass          # context already torn down by the signal handler
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

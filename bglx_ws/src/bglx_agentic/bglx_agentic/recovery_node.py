"""Get the vehicle unstuck when the explorer cannot.

WHY THIS EXISTS
The frontier explorer can only speak in goals. When a plan fails it selects a
different frontier, which fails identically, because the vehicle's POSE is the
problem and not its destination. Observed directly: nine consecutive goals at
bearings spanning 180 degrees, every one stalling at 0.00m, while the trike sat
wedged against a bench. The explorer blacklisted nine perfectly good frontiers
on false evidence and declared exploration complete.

An agent that can read a failure and reposition does not have that problem. It
was observed doing exactly this by hand: fail, read "back off and re-approach
from further out", reverse 3m, retry, reverse 4m, retry, and arrive. Three
recovery cycles, unprompted.

This node puts that capability in the exploration loop WITHOUT putting a
language model in the driving loop.

COST
The model is invoked on exception, not per cycle. Exploration is Nav2 plus a
frontier heuristic, both free; recovery costs a handful of calls only when
something has already gone wrong. Budget is deliberately small: enough to
check, reposition, verify and report, not enough to wander.

AUTHORITY
Recovery may drive, but only after this node has independently confirmed the
vehicle is genuinely stuck. A stall message is a request, not a licence: the
node re-measures before acting, so a spurious or stale request cannot put an
LLM in charge of a moving vehicle. The tool set is narrowed to escape
manoeuvres - no long-range navigation, no landmarks, no map queries - because
the only question here is how to get out of this spot.
"""

import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Header

from bglx_msgs.msg import RecoveryRequest, RecoveryResult

from .agent_loop import (BACKEND, OLLAMA_MODEL, OPENAI_MODEL, ANTHROPIC_MODEL,
                         Agent, AnthropicBackend, OllamaBackend,
                         OpenAICompatBackend, to_openai_tools)
from .robot_tools import RobotTools, _spin
from .tool_schemas import TOOLS
from .trace_log import TraceLog

RECOVERY_BUDGET = 10

STUCK_CONFIRM_DIST = 0.30
STUCK_CONFIRM_SECS = 3.0

RECOVERY_TOOL_NAMES = (
    "get_pose",
    "get_scan_summary",
    "check_attitude",
    "check_turn_around",
    "check_width",
    "get_last_failure",
    "navigate_relative",
    "turn_by",
    "drive",
    "stop",
)

RECOVERY_PROMPT = """You are recovering a stuck autonomous delivery tricycle.

An automatic explorer was driving it and has stopped: the vehicle is not
moving, and trying different destinations does not help because the problem is
WHERE THE VEHICLE IS, not where it was going.

Your only job is to move it somewhere it can drive from again. You are not
exploring, not completing a delivery, and not reaching the original goal.

PLATFORM
- A tricycle with Ackermann steering. It CANNOT rotate in place. It changes
  heading only while moving forward or backward.
- It needs roughly 1.6m of clear space all round to turn around.
- It is 1.55m long and 0.80m wide.
- The LiDAR sees 360 degrees, but only in one horizontal plane at 0.92m. It is
  blind to anything lower - kerbs, steps, dropped objects.

HOW TO WORK
1. Find out where you are and what is around you before moving anything.
2. Work out which direction has room. check_turn_around and check_width answer
   this directly.
3. Move a SHORT distance into that space - two or three metres, not ten. You
   are creating room to manoeuvre, not travelling.
4. Verify with get_pose that the vehicle actually moved. A command that
   reported success but produced no displacement has not worked.
5. Stop as soon as the vehicle is somewhere it can drive from.

CONSTRAINTS
- You have a strict budget of a few tool calls. Do not browse. Every call
  should either gather something you will act on, or move the vehicle.
- Prefer navigate_relative and turn_by. Use drive only if both have failed;
  it has no obstacle avoidance whatsoever.
- If the vehicle is on its side or otherwise unrecoverable, say so and stop.
  Do not keep issuing commands to a vehicle that cannot respond.

WHEN YOU ARE DONE
Finish with a short plain-language account of what you did, and state clearly
whether the original goal is worth retrying or should be abandoned. You have
just been there and looked; the explorer has not."""


def _recovery_tools():
    return [t for t in TOOLS if t["name"] in RECOVERY_TOOL_NAMES]


class RecoveryOllama(OllamaBackend):
    def __init__(self):
        super().__init__()
        self.tools = to_openai_tools(_recovery_tools())

    def start(self, task):
        return [{"role": "system", "content": RECOVERY_PROMPT},
                {"role": "user", "content": task}]


class RecoveryOpenAI(OpenAICompatBackend):
    def __init__(self):
        super().__init__()
        self.tools = to_openai_tools(_recovery_tools())

    def start(self, task):
        return [{"role": "system", "content": RECOVERY_PROMPT},
                {"role": "user", "content": task}]


class RecoveryAnthropic(AnthropicBackend):
    def complete(self, history):
        from .agent_loop import ToolCall
        resp = self.client.messages.create(
            model=ANTHROPIC_MODEL, max_tokens=2000,
            system=RECOVERY_PROMPT, tools=_recovery_tools(), messages=history)
        history.append({"role": "assistant", "content": resp.content})
        text = " ".join(b.text.strip() for b in resp.content
                        if b.type == "text" and b.text.strip())
        calls = [ToolCall(b.name, b.input, b.id)
                 for b in resp.content if b.type == "tool_use"]
        return text, calls


def _make_backend():
    if BACKEND == "ollama":
        return RecoveryOllama()
    if BACKEND == "openai":
        return RecoveryOpenAI()
    return RecoveryAnthropic()


class RecoveryNode(Node):

    def __init__(self, tools, backend, trace):
        super().__init__('bglx_recovery')
        self.tools = tools
        self.backend = backend
        self.trace = trace
        self.cb = ReentrantCallbackGroup()
        self._busy = threading.Lock()

        self.create_subscription(RecoveryRequest, '/bglx/recovery_request',
                                 self._on_request, 10, callback_group=self.cb)
        self.result_pub = self.create_publisher(
            RecoveryResult, '/bglx/recovery_result', 10)

        model = {"ollama": OLLAMA_MODEL, "openai": OPENAI_MODEL,
                 "anthropic": ANTHROPIC_MODEL}.get(BACKEND, "?")
        self.get_logger().info(
            'Recovery standing by (%s / %s, budget %d calls). Listening on '
            '/bglx/recovery_request.' % (BACKEND, model, RECOVERY_BUDGET))

    def _confirm_stuck(self):
        """Independently verify the vehicle is not moving.

        A request is evidence, not authority. Re-measuring here means a stale
        or spurious message cannot hand driving control to a language model.
        """
        first = self.tools._pose()
        if first is None:
            return False, "no pose available"
        time.sleep(STUCK_CONFIRM_SECS)
        second = self.tools._pose()
        if second is None:
            return False, "lost pose while confirming"
        moved = math.hypot(second[0] - first[0], second[1] - first[1])
        if moved > STUCK_CONFIRM_DIST:
            return False, ("vehicle moved %.2fm while confirming - it is not "
                           "stuck, ignoring the request" % moved)
        return True, "confirmed stationary (%.2fm in %.0fs)" % (
            moved, STUCK_CONFIRM_SECS)

    def _on_request(self, msg):
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn('Recovery already running; ignoring.')
            return
        try:
            self._handle(msg)
        except Exception as exc:
            self.get_logger().error('Recovery raised %s: %s'
                                    % (type(exc).__name__, exc))
            self._publish(False, 0.0, 0, True,
                          "recovery failed with an internal error: %s" % exc)
        finally:
            self._busy.release()

    def _handle(self, msg):
        self.get_logger().warn(
            'RECOVERY REQUESTED (%s): stalled %.2fm in %.0fs at (%.2f, %.2f) '
            'heading for (%.2f, %.2f).'
            % (msg.reason, msg.moved_distance, msg.stalled_seconds,
               msg.x, msg.y, msg.goal_x, msg.goal_y))

        upright, attitude = self.tools.check_attitude()
        if upright is False:
            self.get_logger().error('Refusing to drive: %s' % attitude)
            self._publish(False, 0.0, 0, False, attitude)
            return

        ok, why = self._confirm_stuck()
        if not ok:
            self.get_logger().info('Recovery declined: %s' % why)
            self._publish(False, 0.0, 0, True,
                          "recovery not attempted: %s" % why)
            return
        self.get_logger().info('Stuck confirmed: %s' % why)

        before = self.tools._pose()
        task = (
            "The explorer stopped making progress. Reason: %s. It moved only "
            "%.2fm in %.0f seconds while trying to reach (%.2f, %.2f).\n\n"
            "What the navigation stack reported:\n%s\n\n"
            "What the LiDAR saw at that moment:\n%s\n\n"
            "Get the vehicle somewhere it can drive from, then say whether "
            "that goal is worth another attempt."
            % (msg.reason, msg.moved_distance, msg.stalled_seconds,
               msg.goal_x, msg.goal_y,
               msg.diagnosis or "(none recorded)",
               msg.scan_summary or "(none recorded)"))

        agent = Agent(self.tools, self.backend, self.trace)
        import bglx_agentic.agent_loop as al
        saved = al.MAX_STEPS
        try:
            al.MAX_STEPS = RECOVERY_BUDGET
            agent.run(task)
        finally:
            al.MAX_STEPS = saved

        after = self.tools._pose()
        moved = 0.0
        if before and after:
            moved = math.hypot(after[0] - before[0], after[1] - before[1])

        freed = moved > 0.5
        account = "moved %.2fm during recovery. %s" % (
            moved, self.tools.check_turn_around())
        used = self.trace.step if self.trace else 0

        if freed:
            self.get_logger().info('RECOVERY SUCCEEDED: %s' % account)
        else:
            self.get_logger().error(
                'RECOVERY FAILED: the vehicle moved only %.2fm. It may be '
                'physically wedged. Human intervention is likely needed.'
                % moved)

        self._publish(freed, moved, min(used, 255), freed, account)

    def _publish(self, freed, moved, used, retry, account):
        out = RecoveryResult()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.freed = bool(freed)
        out.moved_distance = float(moved)
        out.tool_calls_used = int(used) & 0xFF
        out.goal_worth_retrying = bool(retry)
        out.account = str(account)[:2000]
        self.result_pub.publish(out)


def main(argv=None):
    backend = _make_backend()
    rclpy.init(args=argv)

    tools = RobotTools()
    _spin(tools)
    time.sleep(2.0)

    model = {"ollama": OLLAMA_MODEL, "openai": OPENAI_MODEL,
             "anthropic": ANTHROPIC_MODEL}.get(BACKEND, "?")
    trace = TraceLog(BACKEND, model)

    node = RecoveryNode(tools, backend, trace)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            tools.stop()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

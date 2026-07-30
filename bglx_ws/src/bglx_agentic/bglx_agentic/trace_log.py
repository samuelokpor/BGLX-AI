"""Append-only execution trace.

Every tool call the agent makes is written to JSONL: what was called, with
what arguments, what came back, how long it took, and where the robot was
before and after.

This exists for three reasons:
  1. Primitive usage statistics -- which tools the planner actually reaches
     for, versus which ones we told it to prefer.
  2. Failure evidence. A diagnosis is only worth writing if it changes what
     the agent does next, and that is only measurable across many episodes.
  3. Sim-to-hardware comparison. The same schema logged in Gazebo and on the
     trike is what lets you say how much the diagnoses degrade under real
     noise, rather than asserting that they do.

Traces live in ~/.bglx_traces/ and are never overwritten.
"""

import json
import os
import time
import uuid


DEFAULT_DIR = os.path.expanduser("~/.bglx_traces")


class TraceLog:

    def __init__(self, backend, model, directory=None):
        self.dir = directory or DEFAULT_DIR
        os.makedirs(self.dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(self.dir, "session_%s.jsonl" % stamp)
        self.session_id = uuid.uuid4().hex[:8]
        self.task_id = None
        self.step = 0
        self._write({"type": "session_start", "backend": backend,
                     "model": model})

    def _write(self, record):
        record.setdefault("ts", time.time())
        record.setdefault("session", self.session_id)
        if self.task_id:
            record.setdefault("task_id", self.task_id)
        with open(self.path, "a") as f:
            f.write(json.dumps(record, default=str) + "\n")

    def task_start(self, task):
        self.task_id = uuid.uuid4().hex[:8]
        self.step = 0
        self._write({"type": "task_start", "task": task})

    def task_end(self, reason, steps):
        self._write({"type": "task_end", "reason": reason, "steps": steps})

    def tool_call(self, name, args, result, duration, pose_before, pose_after):
        self.step += 1
        self._write({
            "type": "tool_call",
            "step": self.step,
            "tool": name,
            "args": args,
            "result": result,
            "duration_s": round(duration, 3),
            "pose_before": pose_before,
            "pose_after": pose_after,
            # Cheap flags for later analysis. Deliberately dumb string checks:
            # the point is to find these episodes again, not to classify them.
            "refused": str(result).startswith(("REJECTED", "navigate_to refused",
                                               "navigate_relative refused")),
            "diagnosed": "DIAGNOSIS:" in str(result),
        })

    def agent_text(self, text):
        self._write({"type": "agent_text", "text": text})

    def nudge(self, n):
        self._write({"type": "nudge", "count": n})

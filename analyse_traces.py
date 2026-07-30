"""Summarise BGLX execution traces.

    python3 analyse_traces.py                # all sessions
    python3 analyse_traces.py session_2026*  # a subset
"""

import glob
import json
import math
import os
import sys
from collections import Counter, defaultdict

TRACE_DIR = os.path.expanduser("~/.bglx_traces")


def load(patterns):
    files = []
    for pat in (patterns or ["session_*.jsonl"]):
        files += glob.glob(os.path.join(TRACE_DIR, pat))
    records = []
    for path in sorted(set(files)):
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        records.append(json.loads(line))
                    except json.JSONDecodeError:
                        pass
    return records


def main():
    recs = load(sys.argv[1:])
    if not recs:
        print("No traces found in %s" % TRACE_DIR)
        return

    calls = [r for r in recs if r["type"] == "tool_call"]
    tasks = [r for r in recs if r["type"] == "task_start"]
    ends = [r for r in recs if r["type"] == "task_end"]
    nudges = [r for r in recs if r["type"] == "nudge"]
    sessions = {r["session"] for r in recs}

    print("\n=== SESSIONS ===")
    print("sessions %d   tasks %d   tool calls %d"
          % (len(sessions), len(tasks), len(calls)))
    if ends:
        reasons = Counter(r["reason"] for r in ends)
        print("task outcomes: " + ", ".join("%s %d" % kv
                                            for kv in reasons.items()))
    print("harness nudges: %d" % len(nudges))

    if not calls:
        return

    print("\n=== PRIMITIVE USAGE ===")
    by_tool = Counter(c["tool"] for c in calls)
    dur = defaultdict(list)
    for c in calls:
        dur[c["tool"]].append(c.get("duration_s", 0.0))
    total = len(calls)
    print("%-22s %6s %8s %10s" % ("tool", "calls", "share", "mean s"))
    for tool, n in by_tool.most_common():
        mean = sum(dur[tool]) / len(dur[tool])
        print("%-22s %6d %7.1f%% %10.2f" % (tool, n, 100.0 * n / total, mean))

    # Motion vs sensing: the division-of-labour question.
    motion = {"navigate_to", "navigate_relative", "navigate_to_landmark",
              "drive"}
    sensing = {"get_pose", "get_scan_summary", "list_landmarks",
               "get_last_failure"}
    m = sum(n for t, n in by_tool.items() if t in motion)
    s = sum(n for t, n in by_tool.items() if t in sensing)
    print("\nmotion %d (%.1f%%)   sensing %d (%.1f%%)"
          % (m, 100.0 * m / total, s, 100.0 * s / total))

    print("\n=== SAFETY / LEGIBILITY ===")
    refused = [c for c in calls if c.get("refused")]
    diagnosed = [c for c in calls if c.get("diagnosed")]
    print("refused by tool layer: %d (%.1f%%)"
          % (len(refused), 100.0 * len(refused) / total))
    print("returned a DIAGNOSIS:  %d (%.1f%%)"
          % (len(diagnosed), 100.0 * len(diagnosed) / total))

    blind = by_tool.get("drive", 0)
    print("blind open-loop drive: %d (%.1f%%)"
          % (blind, 100.0 * blind / total))

    print("\n=== DISPLACEMENT ACCURACY ===")
    print("(relative moves only; compares request against measured motion)")
    rows = []
    for c in calls:
        if c["tool"] != "navigate_relative":
            continue
        a, b = c.get("pose_before"), c.get("pose_after")
        if not a or not b:
            continue
        want = math.hypot(float(c["args"].get("forward", 0.0)),
                          float(c["args"].get("left", 0.0)))
        got = math.hypot(b[0] - a[0], b[1] - a[1])
        rows.append((want, got, got - want))
    if rows:
        print("%10s %10s %10s" % ("requested", "actual", "error"))
        for want, got, err in rows:
            print("%10.2f %10.2f %+10.2f" % (want, got, err))
        mean_err = sum(abs(r[2]) for r in rows) / len(rows)
        print("mean absolute error: %.2f m over %d moves" % (mean_err, len(rows)))
    else:
        print("no relative moves logged yet")

    if diagnosed:
        print("\n=== MOST RECENT DIAGNOSES ===")
        for c in diagnosed[-3:]:
            first = str(c["result"]).split("DIAGNOSIS:")[1].strip()
            print("- %s(%s): %s" % (c["tool"], c["args"], first[:160]))


if __name__ == "__main__":
    main()

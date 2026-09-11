
#!/usr/bin/env python3
"""One supervised reverse attempt, then a freshly audited forward attempt."""
import subprocess
import sys
from datetime import datetime
from pathlib import Path

folder = Path(__file__).resolve().parent
logs = Path.home() / "bglx_navtest/logs"
logs.mkdir(parents=True, exist_ok=True)
stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
reverse = folder / "exact_reverse_alignment_motion.py"
forward = folder / "forward_realign_passage.py"

for script in (reverse, forward):
    if not script.is_file():
        raise SystemExit(f"Missing script: {script}")

def run(script, label):
    logfile = logs / f"bounded_{label}_{stamp}.log"
    print(f"\nSTATE {label.upper()}; log={logfile}", flush=True)
    lines = []
    with logfile.open("w") as output:
        process = subprocess.Popen(
            [sys.executable, "-u", str(script)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            for line in process.stdout:
                print(line, end="", flush=True)
                output.write(line)
                output.flush()
                lines.append(line)
            code = process.wait()
        except KeyboardInterrupt:
            # Ctrl+C also reaches the child in this foreground process group.
            # Let its cancellation/stop cleanup finish; never start another stage.
            print("\nOperator cancelled. Waiting for child cleanup.", flush=True)
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                print(
                    "Cleanup not confirmed. Check robot stop and navigation.",
                    flush=True)
            raise SystemExit(130)
    return code, "".join(lines)

print(
    "BOUNDED MOTION SEQUENCE: reverse -> stopped -> fresh forward audit",
    flush=True)
code, report = run(reverse, "reverse")

errors = [
    line.strip() for line in report.splitlines()
    if line.startswith("TEST STOPPED:")
]
allowed_endpoint_stop = (
    errors == ["TEST STOPPED: Unexpected forward command during reverse"]
    and "Final action status: 5" in report
)
completed_reverse = (
    not errors
    and "ACHIEVED:" in report
    and "Final action status: 4" in report
)
clean_stop = (
    code == 0
    and "Odometry confirms stopped: True" in report
    and "Original controller parameters restored." in report
    and "CLEANUP INCOMPLETE:" not in report
    and "Traceback" not in report
)
if not clean_stop or not (completed_reverse or allowed_endpoint_stop):
    raise SystemExit(
        "Sequence stopped after reverse; no forward stage started.")

print(
    "\nSTATE REAUDIT_ACTUAL_POSE: forward script measures the scene again.",
    flush=True)
code, report = run(forward, "forward")
if (
    code != 0
    or "PASSAGE COMPLETE:" not in report
    or "TEST STOPPED:" in report
    or "CLEANUP INCOMPLETE:" in report
    or "Original controller parameters restored." not in report
):
    raise SystemExit(
        "Sequence stopped after forward attempt. No automatic repeat.")

print("\nSEQUENCE COMPLETE: reverse and forward passage finished.", flush=True)

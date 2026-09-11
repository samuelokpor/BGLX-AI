import subprocess
import sys
from datetime import datetime
from pathlib import Path

folder = Path(__file__).resolve().parent
logs = Path.home() / "bglx_navtest/logs"
logs.mkdir(parents=True, exist_ok=True)
stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

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

code, report = run(folder / "mirror_scene.py", "mirror_setup")
if code != 0 or "MIRROR SCENE READY" not in report:
    raise SystemExit("Scene setup failed; no driving started.")

code, report = run(folder / "forward_realign_passage.py", "mirror_forward")
clean = (
    code == 0
    and "Odometry confirms stopped: True" in report
    and "Original controller parameters restored." in report
    and "CLEANUP INCOMPLETE:" not in report
    and "Traceback" not in report
)
errors = [line.strip() for line in report.splitlines()
          if line.startswith("TEST STOPPED:")]

if clean and not errors and "PASSAGE COMPLETE:" in report:
    print("MIRROR COMPLETE: forward passage succeeded.", flush=True)
    raise SystemExit(0)

recoverable = (
    clean and len(errors) == 1
    and errors[0].startswith((
        "TEST STOPPED: Clearance guard:",
        "TEST STOPPED: Tracking guard:",
    ))
)
if not recoverable:
    raise SystemExit(
        "Mirror stopped; result does not permit automatic retreat.")

print("MIRROR RECOVERY: one bounded reverse-and-forward attempt",
      flush=True)
code, report = run(
    folder / "bounded_realign_sequence.py", "mirror_recovery")
if code != 0 or "SEQUENCE COMPLETE:" not in report:
    raise SystemExit("Mirror recovery stopped; no further retry.")
print("MIRROR COMPLETE: recovery and passage succeeded.", flush=True)

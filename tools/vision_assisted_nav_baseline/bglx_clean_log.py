import json
import re
import sys
from datetime import datetime
from pathlib import Path

folder = Path.home() / "bglx_navtest/logs"
folder.mkdir(parents=True, exist_ok=True)
log = folder / ("terminal_full_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".log")
print("Full terminal log:", log, flush=True)

hidden_events = {
    "dependency", "live_openings", "stationary_openings",
    "configuration",
}

with log.open("w", buffering=1) as saved:
    for line in sys.stdin:
        saved.write(line)
        text = line.strip()
        if not text:
            continue

        if text.startswith(("[existing-mission]", "[live-vision]",
                            "[passage-plan]", "[vision-audit]")):
            try:
                data = json.loads(text[text.index("{"):])
            except (ValueError, json.JSONDecodeError):
                print(text, flush=True)
                continue

            event = data.get("event", data.get("strategy", "status"))
            if event in hidden_events:
                continue
            if event == "vision_capture":
                text = "VISION: capturing image and checking openings..."
            elif event == "vision_ranking":
                rank = data.get("ranking", {})
                text = f"VISION: ranked {rank.get('ranking', [])} — {rank.get('reason', rank.get('fallback_reason', ''))}"
            elif event == "vision_candidate_audit":
                text = (
                    f"CHECK {data.get('candidate_id')}: "
                    f"{'PASS' if data.get('passed') else 'FAIL'} | "
                    f"backup needed: {data.get('backup_needed')} "
                    f"{data.get('reason') or ''}"
                )
            elif event == "vision_selected":
                text = f"VISION: selected {data.get('candidate_id')}"
            elif "path_length_m" in data:
                text = f"PATH: {event}, {data['path_length_m']} m"
            else:
                details = {
                    k: v for k, v in data.items()
                    if k not in ("event", "wall_time")
                }
                text = f"{event.upper()}: {json.dumps(details)}"

        elif "Planner loop missed its desired rate" in text:
            continue
        elif re.match(r"pose=.*remaining=", text):
            continue
        elif text.startswith(("/planner_server : active",
                              "/controller_server : active",
                              "/collision_monitor : active")):
            continue
        elif text == "[mission-supervisor] STOP CONFIRMED by fresh odometry":
            continue
        elif set(text) == {"-"}:
            continue

        print(text, flush=True)

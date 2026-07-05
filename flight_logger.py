"""
Flight logger.

Every run creates logs/run_YYYYMMDD_HHMMSS/ containing:
  - state.jsonl : timestamped snapshots of everything (pose, race, command)
  - frames/     : periodic raw camera frames (from vision_rx)

When something goes wrong you send state.jsonl to Claude instead of a
screenshot, and the diagnosis takes one message instead of ten.
"""

import json
import os
import time
from datetime import datetime

import config


def new_run_dir():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(config.LOG_DIR, f"run_{ts}")
    os.makedirs(run_dir, exist_ok=True)
    return run_dir


class FlightLogger:
    def __init__(self, state, run_dir):
        self.state = state
        self.file = open(os.path.join(run_dir, "state.jsonl"), "w")
        self._last_log = 0.0
        self._last_console = 0.0

    def close(self):
        self.file.close()

    def tick(self):
        now = time.time()
        snap = self.state.snapshot()

        if now - self._last_log >= 1.0 / config.LOG_STATE_HZ:
            self._last_log = now
            self._write(now, snap)

        if now - self._last_console >= config.CONSOLE_STATUS_S:
            self._last_console = now
            self._console(snap)

    def _write(self, now, snap):
        race = snap["race"]
        det = snap["detection"]
        rec = {
            "t": round(now, 3),
            "armed": snap["armed"],
            "race_started": race.started if race else None,
            "race_finished": race.finished if race else None,
            "gate_idx": race.active_gate_index if race else None,
            "sim_ms": race.sim_boot_time_ms if race else None,
            "start_ms": race.race_start_boot_time_ms if race else None,
            "det": {
                "cx": round(det.cx, 1), "cy": round(det.cy, 1),
                "w": round(det.w, 1), "h": round(det.h, 1),
                "n": det.n_candidates,
            } if det else None,
            "det_age": round(now - snap["detection_time"], 2) if snap["detection_time"] else None,
            "frames": snap["frame_count"],
            "collisions": snap["n_collisions"],
            "cmd": snap["last_cmd"],
        }
        self.file.write(json.dumps(rec) + "\n")
        self.file.flush()

    def _console(self, snap):
        race = snap["race"]
        det = snap["detection"]

        if race is None:
            race_s = "race:none"
        elif race.finished:
            race_s = "race:FINISHED"
        elif race.started:
            race_s = f"race:GO gate {race.active_gate_index}"
        else:
            race_s = (f"race:countdown/hold (sim_ms={race.sim_boot_time_ms} "
                      f"start_ms={race.race_start_boot_time_ms})")

        if det is not None:
            age = time.time() - snap["detection_time"]
            det_s = f"gate@({det.cx:.0f},{det.cy:.0f}) w{det.w:.0f}px n={det.n_candidates} age{age:.1f}s"
        else:
            det_s = "no gate in view"

        cmd = snap["last_cmd"]
        mode = cmd["mode"] if cmd and "mode" in cmd else "?"
        why = f" | {cmd['why']}" if cmd else ""

        print(f"[{'ARM' if snap['armed'] else 'dis'}] {mode:6s} | {race_s} | {det_s} | "
              f"frames {snap['frame_count']} | hits {snap['n_collisions']}{why}", flush=True)

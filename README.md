# Autonomous Drone Racing Stack — Anduril AI Grand Prix

Autonomous flight stack built in Python for the AI Grand Prix, Anduril Industries' global drone autonomy competition. Solo entry — advanced through Virtual Qualifier 1, currently competing in Virtual Qualifier 2.

## How It Flies

The simulator provides no odometry and no gate positions — so the stack opens with a tuned, scripted racing line, then flies like an FPV pilot: find the gate in the camera, keep it centered, and fly toward it. All velocity commands are in the drone's body frame; the system never needs to know where it is, only where the next gate is in the image.

**Gate detection** (`gate_vision.py`) — OpenCV pipeline detecting the course's glowing gates via HSV color thresholding, tuned against real simulator frames. `detect_gate()` is a pure function (image in, detection out), so detection can be tested on saved frames without running the simulator.

**Visual-servoing controller** (`controller.py`) — a three-state machine:
- `SEARCH` — no gate visible: gentle climb + slow yaw scan until one appears
- `TRACK` — gate visible: yaw to center it, fly forward, climb/descend along the line of sight (accounting for the camera's 20° up-tilt)
- `COMMIT` — gate fills the frame: stop steering and punch straight through, then search for the next gate

Gate passage is confirmed via the race-status gate index, which also ends `COMMIT` early.

**Playbook opening** (`config.py`) — the course is identical every run, so the opening is designed as a sequence of timed "pace notes" (body-frame velocity + pinned thrust), tuned run-over-run from flight logs. Each step advances on elapsed time *or* a gate-pass event, whichever comes first, and vision takes over the moment a real gate looms close — or when the script is exhausted. Playbook state resets automatically when a new race is detected (fixes a bug where a lingering process carried an exhausted script into later runs, silently reverting them to pure-vision flying).

**Course-line following** — when the course's guide line is visible, it takes priority over distant gate detections, preventing far-away gate blips from hijacking control into a wrong-gate, angled approach.

**Telemetry & timing** (`mavlink_rx.py`, `vision_rx.py`, `timesync.py`, `state.py`) — pymavlink-based receivers for telemetry and camera frames, with time synchronization and a shared state object feeding the controller.

**Diagnostics first** (`main.py --diagnose`) — before flying on any new simulator or course, a 30-second listen-only mode reports exactly which data streams are arriving (odometry? track data? race status? camera?). 

**Calibration probe** — an ACRO-mode routine that measures what the controller needs before racing: IMU axis conventions, liftoff/hover thrust (found by using the gate's motion in the camera as an improvised altimeter), and yaw sign.

**Flight logging** (`flight_logger.py`) — per-run log directories capturing telemetry and detections for post-flight review.

## Running It

```bash
pip install -r requirements.txt

python main.py --diagnose   # listen for 30s, report available data streams
python main.py              # fly
```

## Built With

Python, pymavlink, OpenCV, NumPy

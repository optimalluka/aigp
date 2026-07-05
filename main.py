"""
AI Grand Prix autonomy stack -- fresh build.

Usage:
  python main.py              -> fly (waypoint chaser)
  python main.py --diagnose   -> DON'T fly; just listen for 30s and report
                                 exactly which data the sim is sending us.

ALWAYS run --diagnose first on a new simulator/course. It answers, with
zero ambiguity: is odometry arriving? track data? race status? camera?
"""

import argparse
import sys
import time

from pymavlink import mavutil

import config

VERSION = "AIGP VISION STACK v7.8 (per-race playbook reset)"
from state import State
from mavlink_rx import MAVLinkRX
from vision_rx import VisionRX
from timesync import TimeSync
from controller import Controller
from flight_logger import FlightLogger, new_run_dir


def nudge():
    """ACRO CALIBRATION PROBE.
    Measures the things the ACRO controller needs to know:
      0. raw at-rest IMU (axis conventions)
      1. arming with throttle at minimum (the 'put throttle down' fix)
      2. thrust ramp -> finds liftoff/hover thrust using the gate's motion
         in the camera as the altimeter
      3. yaw pulse -> yaw sign check
    Run in a TRAINING flight: start this, then start the race.
    """
    from controller import Controller
    import statistics

    state = State()
    conn = connect()
    MAVLinkRX.start(conn, state)
    TimeSync(conn)
    run_dir = new_run_dir()
    VisionRX(state, run_dir)
    ctrl = Controller(conn, state, int(time.time() * 1000))

    def det_cy(seconds=0.4):
        ys, xs = [], []
        t0 = time.time()
        while time.time() - t0 < seconds:
            s = state.snapshot()
            d = s["detection"]
            if d is not None and time.time() - s["detection_time"] < 0.3:
                ys.append(d.cy); xs.append(d.cx)
            time.sleep(0.02)
        if not ys:
            return None
        return (statistics.median(ys), statistics.median(xs))

    def rates(rr, pr, yr, thrust, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            ctrl.send_attitude_rates(rr, pr, yr, thrust)
            time.sleep(1.0 / config.CONTROL_HZ)

    print("\n========== ACRO CALIBRATION PROBE ==========", flush=True)
    print("Waiting for race GO + a visible gate...", flush=True)
    ctrl.arm()
    while True:
        s = state.snapshot()
        race_ok = s["race"] is not None and s["race"].started
        det_ok = s["detection"] is not None and time.time() - s["detection_time"] < 0.3
        if race_ok and det_ok:
            break
        # throttle at MINIMUM while waiting: this is the FPV arm requirement
        ctrl.send_attitude_rates(0, 0, 0, 0.0)
        time.sleep(0.05)

    s = state.snapshot()
    print("\n--- 0: at-rest IMU (tell Claude these numbers) ---", flush=True)
    print(f"  accel: {s['imu_accel']}", flush=True)
    print(f"  gyro:  {s['imu_gyro']}", flush=True)
    print(f"  est roll/pitch: {s['est_roll']:.3f} / {s['est_pitch']:.3f} rad", flush=True)
    print(f"  motors: {s['motors']}  armed: {s['armed']}", flush=True)

    print("\n--- 1: thrust ramp (gate cy = altimeter; cy INCREASING = climbing) ---", flush=True)
    hover_guess = None
    base = det_cy()
    for thrust in [0.2, 0.3, 0.4, 0.5, 0.6, 0.7]:
        rates(0, 0, 0, thrust, 1.2)
        now_cy = det_cy()
        s = state.snapshot()
        m = s["motors"]
        if base is None or now_cy is None:
            print(f"  thrust {thrust:.1f}: gate not visible (motors {m})", flush=True)
        else:
            dcy = now_cy[0] - base[0]
            note = ""
            if dcy > 12 and hover_guess is None:
                hover_guess = thrust
                note = "  <-- LIFTOFF around here"
            print(f"  thrust {thrust:.1f}: gate cy moved {dcy:+.0f}px (motors {m}){note}", flush=True)
            base = now_cy
        if hover_guess is not None:
            break

    print("\n--- 2: settle at estimated hover ---", flush=True)
    hover = hover_guess if hover_guess is not None else config.HOVER_THRUST
    rates(0, 0, 0, hover, 1.0)

    print("--- 3: yaw pulse +0.8 rad/s (gate should move LEFT if yaw sign correct) ---", flush=True)
    before = det_cy()
    rates(0, 0, 0.8, hover, 0.8)
    after = det_cy()
    if before and after:
        dcx = after[1] - before[1]
        verdict = "CORRECT" if dcx < -8 else ("INVERTED -> set SIGN_YAW = -1.0" if dcx > 8 else "weak/none")
        print(f"  gate cx moved {dcx:+.0f}px -> yaw {verdict}", flush=True)
    else:
        print("  gate not visible during yaw test", flush=True)

    rates(0, 0, 0, 0.0, 0.5)
    print("\n============ RESULTS ============", flush=True)
    if hover_guess is not None:
        print(f"  LIFTOFF THRUST ~ {hover_guess:.1f}")
        print(f"  >>> notepad config.py  ->  HOVER_THRUST = {max(0.2, hover_guess - 0.05):.2f}")
    else:
        print("  liftoff not confirmed by camera; send Claude the full output + frames")
    print("  Copy this ENTIRE output to Claude.")
    print("=================================\n", flush=True)


def connect():
    conn = mavutil.mavlink_connection(
        f"udpin:{config.SIM_SERVER_UDP_IP}:{config.SIM_SERVER_UDP_PORT}")
    print("Waiting for heartbeat from simulator (is FlightSim running and in a course?)...", flush=True)
    conn.wait_heartbeat()
    print(f"Connected to system {conn.target_system}", flush=True)
    return conn


def diagnose():
    state = State()
    run_dir = new_run_dir()
    conn = connect()
    MAVLinkRX.start(conn, state)
    TimeSync(conn)
    VisionRX(state, run_dir)

    print("\n--- DIAGNOSE MODE: listening for 30 seconds, sending nothing ---\n", flush=True)
    t0 = time.time()
    while time.time() - t0 < 30:
        time.sleep(5)
        snap = state.snapshot()
        print(f"  msg counts so far: {snap['msg_counts']} | camera frames: {snap['frame_count']}", flush=True)

    snap = state.snapshot()
    print("\n================ DIAGNOSIS ================")
    checks = [
        ("HEARTBEAT (sim alive)",        snap["msg_counts"].get("HEARTBEAT", 0) > 0),
        ("ODOMETRY (our pose)",          snap["msg_counts"].get("ODOMETRY", 0) > 0),
        ("LOCAL_POSITION_NED",           snap["msg_counts"].get("LOCAL_POSITION_NED", 0) > 0),
        ("ATTITUDE",                     snap["msg_counts"].get("ATTITUDE", 0) > 0),
        ("Race status (encapsulated)",   snap["race"] is not None),
        ("TRACK DATA (gate positions)",  len(snap["gates"]) > 0),
        ("Camera frames",                snap["frame_count"] > 0),
    ]
    for name, ok in checks:
        print(f"  [{'OK ' if ok else 'MISSING'}] {name}")
    if snap["race"] is not None:
        print(f"  race started: {snap['race'].started}, active gate: {snap['race'].active_gate_index}")
    print(f"  full message counts: {snap['msg_counts']}")
    print("===========================================\n")
    print(f"Log dir: {run_dir}")
    print("Send this whole output to Claude.")


def fly():
    state = State()
    run_dir = new_run_dir()
    print(f"Logging to {run_dir}", flush=True)

    conn = connect()
    system_boot_ms = int(time.time() * 1000)

    MAVLinkRX.start(conn, state)
    TimeSync(conn)
    VisionRX(state, run_dir)
    controller = Controller(conn, state, system_boot_ms)
    logger = FlightLogger(state, run_dir)

    print("Entering control loop (Ctrl+C to stop)...", flush=True)
    import traceback
    last_err_print = 0.0
    try:
        while True:
            try:
                controller.tick()
                logger.tick()
            except Exception:
                # NEVER die silently mid-flight: the sim latches the last
                # command. Print, send a safe hover, keep looping.
                if time.time() - last_err_print > 1.0:
                    last_err_print = time.time()
                    traceback.print_exc()
                try:
                    controller.send_attitude_rates(0, 0, 0, controller._hover_trim)
                except Exception:
                    pass
            time.sleep(1.0 / config.CONTROL_HZ)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        logger.close()
        print(f"Run log saved: {run_dir}/state.jsonl")


if __name__ == "__main__":
    print(VERSION, flush=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("--diagnose", action="store_true",
                        help="listen only; report what the sim sends")
    parser.add_argument("--nudge", action="store_true",
                        help="ACRO calibration probe: hover thrust + axis signs (run in Training)")
    args = parser.parse_args()
    if args.diagnose:
        sys.exit(diagnose())
    elif args.nudge:
        sys.exit(nudge())
    else:
        sys.exit(fly())

"""
Visual-servoing controller (FlightSim 2.0: no odometry, no gate positions).

Since the sim gives us no world position, we fly like an FPV pilot:
keep the gate centered in the camera and move toward it. All velocity
commands are in the drone's BODY frame, so we never need to know where
we are -- only where the gate is in the image.

State machine:
  SEARCH -> no gate visible: gentle climb + slow yaw scan until one appears
  TRACK  -> gate visible: yaw to center it, fly forward, climb/descend
            along the line of sight (camera's 20 deg up-tilt accounted for)
  COMMIT -> gate very close (fills the frame): stop steering, punch
            straight through for a fixed time, then search for the next one

Gate passage is confirmed by race status active_gate_index incrementing,
which also ends COMMIT early.
"""

import json
import math
import os
import time

from pymavlink import mavutil

import config
from state import State

MAVLINK_CMD_SIM_RESET = 31000

# Keep vx, vy, vz and yaw_rate active; ignore position, accel, absolute yaw.
VEL_YAWRATE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Controller:
    def __init__(self, conn, state: State, system_boot_ms: int):
        self.conn = conn
        self.state = state
        self.system_boot_ms = system_boot_ms
        self._last_arm_attempt = 0.0
        self.mode = "SEARCH"
        self._commit_until = 0.0
        self._last_gate_idx = None
        # memory of the last good detection, for smart SEARCH
        self._last_seen = None       # (norm_x, norm_y) in [-1,1], + = right/below
        self._last_seen_t = 0.0
        # takeoff + anti-grind state
        self._launched = False
        self._launch_until = 0.0
        self._start_field_seen_t = None   # when race_start first became >0
        self._go_fallback_used = False
        self._unstick_until = 0.0
        self._col_ref_t = 0.0
        self._col_ref_n = 0
        # learned hover trim (starts at the configured guess, then adapts)
        self._hover_trim = config.HOVER_THRUST
        self._trim_last_t = None
        self._last_thrust = 0.0
        # visual thrust integrator (camera-truth altitude authority)
        self._el_bias = 0.0
        self._el_last_t = None
        # in-flight sign auto-calibration (command -> gyro response)
        self._cmd_sign = {"roll": 1.0, "pitch": 1.0, "yaw": 1.0}
        self._calib_plan = []        # list of (axis, t_start, t_end)
        self._calib_readings = {}    # axis -> list of gyro readings
        self._calib_done = not config.CALIB_ENABLE
        # forward-direction self-check
        self._w_hist = []            # (t, gate width px)
        self._fwd_sign = 1.0
        self._last_fwd_flip = 0.0
        # yaw-steering self-check (camera-verified steering direction)
        # default -1: every camera-verified measurement in this sim was -1
        self._steer_yaw_sign = -1.0
        self._az_hist = []           # (t, az rad)
        self._last_yaw_flip = 0.0
        # ping-pong detector (fast same-side TRACK losses)
        self._prev_mode = "HOLD"
        self._carry_until = 0.0
        self._track_since = None
        self._last_track_az = 0.0
        self._pingpong_events = []   # (t, side)
        # committed scan direction
        self._scan_dir = 0.0
        self._scan_dir_until = 0.0
        self._nomem_since = None
        # optical-flow motion check
        self._flow_hist = []         # (t, |dy|, vz_intent)
        self._flow_nudges = 0
        self._last_flow_nudge = 0.0
        self._flow_last_vz_sign = 0
        # signed flow: learned during launch boost (we KNOW we're climbing)
        self._flow_up_sign = 0.0     # 0 = unknown / untrusted
        self._flow_sign_votes = []   # recent candidate up-signs
        self._launch_flow = []
        self._drift_hist = []        # (t, dy) while commanding hover
        self._last_drift_nudge = 0.0
        self._drift_nudges = 0
        self._drift_last_mag = None
        # camera-verified vertical trim
        self._el_hist = []           # (t, el_body, vz_intent)
        self._last_vert_nudge = 0.0
        self._vert_nudge_streak = 0
        self._load_cache()

    # ------------------------------------------------------------- commands
    def arm(self):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)

    def sim_reset(self):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            MAVLINK_CMD_SIM_RESET,
            0, 0, 0, 0, 0, 0, 0, 0)

    def _send_body_velocity(self, vx, vy, vz, yaw_rate, why, grounded=False):
        """Abstract intent: vx m/s forward, vz m/s down(+), yaw_rate rad/s.
        Dispatched to whichever backend CONTROL_MODE selects.
        grounded=True means "we are (or should be) on the ground with props
        at minimum" -- in ACRO this sends thrust 0 (satisfies the FPV
        'throttle down to arm' safety)."""
        now_ms = int(time.time() * 1000)
        try:
            if config.CONTROL_MODE == "ACRO":
                self._acro_send(vx, vy, vz, yaw_rate, grounded)
            else:
                self._raw_send(now_ms, vx, vy, vz, yaw_rate)
        except (ConnectionResetError, OSError):
            pass  # sim port briefly closed; keep looping
        self.state.set_last_cmd({
            "t": time.time(), "mode": self.mode,
            "vx": round(vx, 2), "vy": round(vy, 2), "vz": round(vz, 2),
            "yaw_rate": round(yaw_rate, 2),
            "thrust": round(self._last_thrust, 3),
            "trim": round(self._hover_trim, 3),
            "why": why,
        })

    def _acro_send(self, vx, vy, vz, yaw_rate, grounded):
        """Translate intents into ACRO body rates + thrust with an
        angle-hold loop on top of the IMU attitude estimate."""
        if grounded:
            self.send_attitude_rates(0.0, 0.0, 0.0, config.THRUST_MIN)
            return
        snap = self.state.snapshot()
        est_roll = snap["est_roll"]
        est_pitch = snap["est_pitch"]

        # forward intent -> target pitch angle (nose down = forward)
        pitch_target = config.SIGN_PITCH_FWD * self._fwd_sign * clamp(
            config.PITCH_PER_MS * vx, -config.MAX_PITCH, config.MAX_PITCH)
        # sideways intent -> bank angle: this is what lets the drone STRAFE
        # toward an off-center gate instead of orbiting it with yaw alone
        roll_target = clamp(config.ROLL_PER_MS * vy,
                            -config.MAX_ROLL, config.MAX_ROLL) if config.STRAFE_ENABLE else 0.0

        roll_rate = clamp(config.ANGLE_KP * (roll_target - est_roll) * config.SIGN_ROLL,
                          -config.MAX_BODY_RATE, config.MAX_BODY_RATE)
        pitch_rate = clamp(config.ANGLE_KP * (pitch_target - est_pitch),
                           -config.MAX_BODY_RATE, config.MAX_BODY_RATE)

        # vertical: CLOSED LOOP. thrust chases the vz target using the
        # accel-derived vertical velocity; integral term learns true hover.
        vz_est = snap["est_vz"]
        err = vz_est - vz            # + means descending faster than wanted
        now_t = time.time()
        thrust = clamp(self._hover_trim + self._el_bias + config.VZ_KP * err,
                       config.THRUST_MIN, config.THRUST_MAX)
        # saturation override: if we are asking for (near) max climb, the
        # closed loop must not be allowed to sit at hover-ish thrust -- the
        # v5.8 logs show el worsening for many seconds at max climb intent.
        if vz <= -0.9 * config.MAX_VZ:
            thrust = max(thrust, config.CLIMB_SAT_THRUST)
        # FLOW DAMPING: without odometry we cannot feel vertical speed, but
        # fast image flow while climbing = moving fast -> back off thrust
        # BEFORE overshooting instead of after.
        if vz < -0.2:
            fdy = abs(getattr(self, "_last_flow_dy", 0.0))
            thrust -= min(config.FLOW_DAMP_K * fdy, config.FLOW_DAMP_MAX)
            thrust = clamp(thrust, config.THRUST_MIN, config.THRUST_MAX)
        # opening window: thrust PINNED (not just capped) for a fully
        # deterministic start -- adaptive trim/flow nudges caused run-to-run
        # variance (toad-hop vs balloon) with an identical script
        pin = getattr(self, "_pb_pin", None)
        if pin is not None:
            thrust = pin
        # Learn hover trim ONLY near hover intent and away from saturation:
        # during commanded climbs/descents the leaky vz estimate underreads
        # and would ratchet the trim into the ground (run 20260703: trim
        # collapse during long descend-scan made TRACK unable to climb).
        learning = (abs(vz) <= config.TRIM_LEARN_VZ_MAX and
                    config.THRUST_MIN + 0.02 < thrust < config.THRUST_MAX - 0.02)
        if learning and self._trim_last_t is not None:
            dt = min(0.1, now_t - self._trim_last_t)
            self._hover_trim = clamp(self._hover_trim + config.TRIM_KI * err * dt,
                                     config.TRIM_MIN, config.TRIM_MAX)
        self._trim_last_t = now_t
        self._last_thrust = thrust
        self.send_attitude_rates(roll_rate * self._cmd_sign["roll"],
                                 pitch_rate * self._cmd_sign["pitch"],
                                 yaw_rate * config.SIGN_YAW * self._cmd_sign["yaw"],
                                 thrust)

    def _raw_send(self, now_ms, vx, vy, vz, yaw_rate):
        vx *= config.SIGN_VX
        vz *= config.SIGN_VZ
        yaw_rate *= config.SIGN_YAW
        self.conn.mav.set_position_target_local_ned_send(
            now_ms - self.system_boot_ms,
            self.conn.target_system, self.conn.target_component,
            config.BODY_FRAME,
            VEL_YAWRATE_MASK,
            0.0, 0.0, 0.0,       # position ignored
            vx, vy, vz,          # velocity, BODY frame (x fwd, y right, z down)
            0.0, 0.0, 0.0,       # accel ignored
            0.0,                 # yaw ignored
            yaw_rate)

    def send_attitude_rates(self, roll_rate, pitch_rate, yaw_rate, thrust):
        """SET_ATTITUDE_TARGET with body rates + thrust (spec's 2nd interface)."""
        now_ms = int(time.time() * 1000)
        mask = mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
        try:
            self.conn.mav.set_attitude_target_send(
                now_ms - self.system_boot_ms,
                self.conn.target_system, self.conn.target_component,
                mask,
                [1, 0, 0, 0],          # quaternion ignored
                roll_rate, pitch_rate, yaw_rate,
                thrust)
        except (ConnectionResetError, OSError):
            pass

    def _launch_total(self):
        """Total planned launch duration (base + calibration extension)."""
        if self._calib_plan:
            return max(config.LAUNCH_TIME_S,
                       self._calib_plan[-1][2] - (self._calib_plan[0][1] - 0.4) + 0.1)
        return config.LAUNCH_TIME_S

    def _finish_calibration(self):
        self._calib_done = True
        if len(self._launch_flow) >= 8:
            mean_dy = sum(self._launch_flow) / len(self._launch_flow)
            if abs(mean_dy) > 0.15:
                self._flow_sign_vote(1.0 if mean_dy > 0 else -1.0, "launch climb")
        self._save_cache()
        for axis in ("roll", "pitch", "yaw"):
            readings = self._calib_readings.get(axis, [])
            if not readings:
                continue
            # skip the first samples (actuation delay), average the rest
            usable = readings[len(readings) // 3:]
            mean = sum(usable) / max(1, len(usable))
            if abs(mean) >= config.CALIB_MIN_RESPONSE:
                sign = 1.0 if mean > 0 else -1.0
                self._cmd_sign[axis] = sign
                verdict = "OK (+)" if sign > 0 else "INVERTED -> auto-corrected"
                print(f">> CALIB {axis}: gyro responded {mean:+.2f} rad/s "
                      f"to +{config.CALIB_PULSE_RATE} cmd -> {verdict}", flush=True)
            else:
                print(f">> CALIB {axis}: weak gyro response ({mean:+.2f} rad/s); "
                      f"keeping sign +1", flush=True)
        print(f">> CALIB result: {self._cmd_sign}", flush=True)

    def _load_cache(self):
        try:
            if os.path.exists(config.CALIB_CACHE_FILE):
                with open(config.CALIB_CACHE_FILE) as f:
                    c = json.load(f)
                self._cmd_sign.update(c.get("cmd_sign", {}))
                self._steer_yaw_sign = c.get("steer_yaw_sign", -1.0)
                self._fwd_sign = c.get("fwd_sign", 1.0)
                # trim and flow sign are deliberately NOT loaded: cheap to
                # relearn, catastrophic to inherit wrong (runs 20260704 pm)
                self._calib_done = c.get("calib_done", False) or self._calib_done
                print(f">> loaded learned calibration: cmd_sign={self._cmd_sign} "
                      f"steer={self._steer_yaw_sign:+.0f} fwd={self._fwd_sign:+.0f}", flush=True)
        except Exception as e:
            print(f">> calib cache load failed ({e}); starting fresh", flush=True)

    def _save_cache(self):
        try:
            with open(config.CALIB_CACHE_FILE, "w") as f:
                json.dump({"cmd_sign": self._cmd_sign,
                           "steer_yaw_sign": self._steer_yaw_sign,
                           "fwd_sign": self._fwd_sign,
                           "calib_done": self._calib_done}, f)
        except Exception:
            pass

    def _pingpong_check(self, now):
        """Called on TRACK->lost transitions. Two fast same-side losses in a
        row = steering is expelling the gate from view: flip immediately."""
        if self._calib_done and not config.ALLOW_SIGN_FLIPS:
            return   # calibration is trusted; detector noise must not un-learn it
        if self._track_since is None:
            return
        duration = now - self._track_since
        self._track_since = None
        if duration >= config.PINGPONG_FAST_LOSS_S:
            self._pingpong_events = []       # a solid track run: not ping-pong
            return
        if abs(self._last_track_az) < config.PINGPONG_SIDE_RAD:
            return                           # lost near center: not a steering issue
        side = 1.0 if self._last_track_az > 0 else -1.0
        self._pingpong_events = [(t, s) for (t, s) in self._pingpong_events
                                 if now - t < config.PINGPONG_RESET_S]
        self._pingpong_events.append((now, side))
        same = [s for (t, s) in self._pingpong_events if s == side]
        if len(same) >= config.PINGPONG_COUNT:
            self._steer_yaw_sign *= -1.0
            self._last_yaw_flip = now
            self._pingpong_events = []
            self._az_hist = []
            self._save_cache()
            print(f">> PING-PONG detected: {config.PINGPONG_COUNT} fast losses to the "
                  f"same side -- steering sign flipped to {self._steer_yaw_sign:+.0f}.", flush=True)

    def _flow_motion_check(self, now, snap, vz):
        """SEARCH-mode vertical truth: commanded vertical motion must make the
        scene stream vertically in the camera. If it doesn't, push the trim
        toward the commanded direction (sign-free: magnitude test only)."""
        if abs(vz) < config.FLOW_MIN_INTENT:
            self._flow_hist = []
            return
        if now - snap["flow_time"] > 0.5:
            return   # no fresh flow (camera stalled); don't judge
        self._flow_hist.append((now, snap["flow_dy"], vz))
        self._flow_hist = [(t, f, v) for (t, f, v) in self._flow_hist
                           if now - t <= config.FLOW_WINDOW_S + 0.3]
        if now - self._last_flow_nudge < config.FLOW_WINDOW_S:
            return
        window = [(t, f, v) for (t, f, v) in self._flow_hist
                  if now - t <= config.FLOW_WINDOW_S]
        if len(window) < 20:
            return
        if not all((v < 0) == (vz < 0) for (_, _, v) in window):
            return   # intent direction changed mid-window
        span = now - window[0][0]
        if span < config.FLOW_WINDOW_S * 0.8:
            return
        # fresh budget every time the commanded direction flips (sweep phases)
        vz_sign = 1 if vz > 0 else -1
        if vz_sign != self._flow_last_vz_sign:
            self._flow_last_vz_sign = vz_sign
            self._flow_nudges = 0

        mean_signed = sum(f for (_, f, _) in window) / len(window)
        mean_flow = sum(abs(f) for (_, f, _) in window) / len(window)
        if mean_flow >= config.FLOW_MIN_PX:
            if abs(mean_signed) >= config.FLOW_MIN_PX * 0.7:
                cand = (-1.0 if mean_signed > 0 else 1.0) if vz > 0 else (1.0 if mean_signed > 0 else -1.0)
                if self._flow_up_sign != 0.0:
                    # sign is trusted: moving is not enough, DIRECTION must match
                    expected = -self._flow_up_sign if vz > 0 else self._flow_up_sign
                    actual = 1.0 if mean_signed > 0 else -1.0
                    if actual != expected and self._flow_nudges < config.FLOW_NUDGE_BUDGET:
                        direction = -1.0 if vz > 0 else +1.0
                        self._hover_trim = clamp(self._hover_trim + direction * config.FLOW_NUDGE,
                                                 config.TRIM_MIN, config.TRIM_MAX)
                        self._flow_nudges += 1
                        # wrong-way evidence is unambiguous: re-arm faster
                        self._last_flow_nudge = now - config.FLOW_WINDOW_S * 0.5
                        self._flow_hist = []
                        print(f">> FLOW CHECK: commanded {'descend' if vz > 0 else 'climb'} but "
                              f"moving the WRONG WAY -- trim {'cut' if vz > 0 else 'raised'} to "
                              f"{self._hover_trim:.2f} ({self._flow_nudges}/{config.FLOW_NUDGE_BUDGET})",
                              flush=True)
                        return
                self._flow_sign_vote(cand, "commanded sweep")
            self._flow_nudges = 0 if self._flow_up_sign == 0.0 else self._flow_nudges
            return
        if self._flow_nudges >= config.FLOW_NUDGE_BUDGET:
            return
        direction = -1.0 if vz > 0 else +1.0    # descend stuck -> less thrust
        self._hover_trim = clamp(self._hover_trim + direction * config.FLOW_NUDGE,
                                 config.TRIM_MIN, config.TRIM_MAX)
        self._flow_nudges += 1
        self._last_flow_nudge = now
        self._flow_hist = []
        self._save_cache()
        print(f">> FLOW CHECK: commanded {'descend' if vz > 0 else 'climb'} but scene "
              f"isn't moving -- trim nudged to {self._hover_trim:.2f} "
              f"({self._flow_nudges}/{config.FLOW_NUDGE_BUDGET})", flush=True)

    def _flow_sign_vote(self, candidate, source):
        """Flow sign must be confirmed twice in a row before it is trusted;
        any contradiction resets trust to unknown."""
        self._flow_sign_votes.append(candidate)
        self._flow_sign_votes = self._flow_sign_votes[-3:]
        if len(self._flow_sign_votes) >= 2 and self._flow_sign_votes[-1] == self._flow_sign_votes[-2]:
            if self._flow_up_sign != candidate:
                self._flow_up_sign = candidate
                print(f">> FLOW SIGN trusted ({source}): climbing => dy "
                      f"{'positive' if candidate > 0 else 'negative'}", flush=True)
        elif self._flow_up_sign != 0.0 and candidate != self._flow_up_sign:
            self._flow_up_sign = 0.0
            print(f">> FLOW SIGN contradiction ({source}); trust reset", flush=True)

    def _hover_drift_check(self, now, snap, vz):
        """While commanding hover, sustained vertical scene flow = drifting.
        Uses the launch-learned flow sign to push trim against the drift."""
        if abs(vz) > 0.2 or self._flow_up_sign == 0.0:
            self._drift_hist = []
            return
        if now - snap["flow_time"] > 0.5:
            return
        self._drift_hist.append((now, snap["flow_dy"]))
        self._drift_hist = [(t, d) for (t, d) in self._drift_hist
                            if now - t <= config.FLOW_WINDOW_S + 0.3]
        if now - self._last_drift_nudge < config.FLOW_WINDOW_S:
            return
        window = [(t, d) for (t, d) in self._drift_hist if now - t <= config.FLOW_WINDOW_S]
        if len(window) < 20 or now - window[0][0] < config.FLOW_WINDOW_S * 0.8:
            return
        mean_dy = sum(d for (_, d) in window) / len(window)
        if abs(mean_dy) < config.HOVER_DRIFT_FLOW_PX:
            self._drift_nudges = 0
            self._drift_last_mag = None
            return
        # if our corrections are making drift WORSE, our sign belief is wrong:
        # stand down and revoke trust rather than fan the flames
        if self._drift_nudges >= 3:
            if self._drift_last_mag is not None and abs(mean_dy) > self._drift_last_mag:
                self._flow_up_sign = 0.0
                self._flow_sign_votes = []
                self._drift_nudges = 0
                print(">> HOVER DRIFT: corrections made it worse -- flow sign "
                      "distrusted, police standing down", flush=True)
            return
        self._drift_last_mag = abs(mean_dy)
        drifting_up = (mean_dy > 0) == (self._flow_up_sign > 0)
        self._hover_trim = clamp(self._hover_trim +
                                 (-config.HOVER_DRIFT_NUDGE if drifting_up else +config.HOVER_DRIFT_NUDGE),
                                 config.TRIM_MIN, config.TRIM_MAX)
        self._last_drift_nudge = now
        self._drift_nudges += 1
        self._drift_hist = []
        print(f">> HOVER DRIFT: {'ascending' if drifting_up else 'sinking'} while commanding "
              f"hover -- trim corrected to {self._hover_trim:.2f}", flush=True)

    def _committed_scan_dir(self, now, preferred):
        """Return a sweep direction that is held for SCAN_COMMIT_S."""
        if now >= self._scan_dir_until or self._scan_dir == 0.0:
            self._scan_dir = preferred if preferred != 0.0 else (self._scan_dir or 1.0)
            self._scan_dir_until = now + config.SCAN_COMMIT_S
        return self._scan_dir

    def _angles(self, det):
        fx = config.CAM_FX * (det.frame_w / 640.0)
        fy = config.CAM_FY * (det.frame_h / 360.0)
        cx0 = det.frame_w / 2.0
        cy0 = det.frame_h / 2.0
        az = math.atan2(det.cx - cx0, fx)
        el_cam = math.atan2(cy0 - det.cy, fy)
        return az, el_cam + config.CAM_TILT_COMP_RAD

    def _yaw_steer_check(self, now, az):
        """If |az| keeps GROWING while we steer toward the gate, the steering
        direction is inverted relative to the camera: flip it once."""
        self._az_hist.append((now, az))
        self._az_hist = [(t, a) for (t, a) in self._az_hist
                         if now - t <= config.YAW_STEER_WINDOW_S + 0.3]
        if self._calib_done and not config.ALLOW_SIGN_FLIPS:
            return   # signs are frozen post-calibration
        if now - self._last_yaw_flip < config.YAW_FLIP_COOLDOWN_S:
            return
        old = [(t, a) for (t, a) in self._az_hist
               if now - t >= config.YAW_STEER_WINDOW_S]
        if not old:
            return
        t0, a0 = old[-1]
        same_side = all((a >= 0) == (az >= 0) for (_, a) in self._az_hist)
        if same_side and abs(az) - abs(a0) >= config.YAW_STEER_GROW_RAD:
            self._steer_yaw_sign *= -1.0
            self._last_yaw_flip = now
            self._az_hist = []
            print(f">> YAW STEER CHECK: gate drifting AWAY while steering at it -- "
                  f"steering sign flipped to {self._steer_yaw_sign:+.0f}.", flush=True)

    def _vertical_check(self, now, el, vz, gate_w=999.0, w_growing=False):
        if w_growing:
            self._vert_nudge_streak = 0
            return   # gate size increasing = closing in; el-constant is FINE
        """Camera-supervised throttle: commanding vertical motion must move
        the gate's row in the image. If it stalls, the hover trim is wrong;
        nudge it in the needed direction until the camera agrees."""
        self._el_hist.append((now, el, vz))
        self._el_hist = [(t, e, v) for (t, e, v) in self._el_hist
                         if now - t <= config.VERT_STALL_WINDOW_S + 0.3]
        if abs(vz) < config.VERT_MIN_INTENT:
            return
        if gate_w < config.VERT_MIN_GATE_W:
            return   # too far away to be evidence either way
        if now - self._last_vert_nudge < config.VERT_STALL_WINDOW_S:
            return
        old = [(t, e, v) for (t, e, v) in self._el_hist
               if now - t >= config.VERT_STALL_WINDOW_S]
        if not old:
            return
        t0, e0, v0 = old[-1]
        # the whole window must have had the same vertical intent direction
        if not all((v < 0) == (vz < 0) and abs(v) >= config.VERT_MIN_INTENT
                   for (_, _, v) in self._el_hist):
            return
        improved = abs(e0) - abs(el)
        if improved >= config.VERT_STALL_EL_RAD:
            self._vert_nudge_streak = 0      # camera confirms progress
            return
        if self._vert_nudge_streak >= config.VERT_NUDGE_BUDGET:
            return   # nudging isn't helping; hold rather than wind to the rails
        direction = +1.0 if vz < 0 else -1.0   # climb stalled -> more thrust
        self._hover_trim = clamp(self._hover_trim + direction * config.VERT_NUDGE,
                                 config.TRIM_MIN, config.TRIM_MAX)
        self._last_vert_nudge = now
        self._vert_nudge_streak += 1
        self._el_hist = []
        self._save_cache()
        print(f">> VERTICAL CHECK: commanded {'climb' if vz < 0 else 'descend'} but "
              f"camera shows no progress -- trim nudged to {self._hover_trim:.2f} "
              f"({self._vert_nudge_streak}/{config.VERT_NUDGE_BUDGET})", flush=True)

    def _forward_check(self, now, det, vx):
        """If we command forward but the gate keeps shrinking, we are flying
        backward: flip the forward-pitch sign once and say so."""
        self._w_hist.append((now, det.w))
        self._w_hist = [(t, w) for (t, w) in self._w_hist
                        if now - t <= config.FWD_CHECK_WINDOW_S + 0.5]
        if self._calib_done and not config.ALLOW_SIGN_FLIPS:
            return   # signs are frozen post-calibration
        if vx < 1.0 or now - self._last_fwd_flip < config.FWD_FLIP_COOLDOWN_S:
            return
        old = [w for (t, w) in self._w_hist if now - t >= config.FWD_CHECK_WINDOW_S]
        if not old:
            return
        if old[-1] - det.w >= config.FWD_CHECK_SHRINK_PX:
            self._fwd_sign *= -1.0
            self._last_fwd_flip = now
            self._w_hist = []
            print(f">> FORWARD CHECK: gate shrinking while commanding forward -- "
                  f"flying BACKWARD. Flipping forward pitch sign to "
                  f"{config.SIGN_PITCH_FWD * self._fwd_sign:+.0f}.", flush=True)

    # ------------------------------------------------------------- main tick
    _race_go_t = None

    def tick(self):
        snap = self.state.snapshot()

        if not snap["armed"] and time.time() - self._last_arm_attempt > config.ARM_RETRY_S:
            self._last_arm_attempt = time.time()
            self.arm()

        race = snap["race"]

        # Decide GO. Three cases:
        #  - no race status yet, or start field unset -> HOLD, send NOTHING
        #    (the sim shows a "throttle down" arming guard; even zero-velocity
        #    setpoints read as mid-throttle, so we stay fully silent pre-GO)
        #  - start field set and sim time passed it -> GO
        #  - start field set >6s ago but comparison never passed (time-base
        #    mismatch failsafe) -> GO anyway; a real countdown is only 3s
        go = False
        if race is not None and race.race_start_boot_time_ms and race.race_start_boot_time_ms > 0:
            if self._start_field_seen_t is None:
                self._start_field_seen_t = time.time()
            if race.started:
                go = True
            elif time.time() - self._start_field_seen_t > 6.0:
                go = True
                if not self._go_fallback_used:
                    self._go_fallback_used = True
                    print(">> GO FAILSAFE: start time set >6s ago but time comparison "
                          f"never passed (sim_boot={race.sim_boot_time_ms}, "
                          f"race_start={race.race_start_boot_time_ms}). Launching anyway.",
                          flush=True)
        else:
            self._start_field_seen_t = None

        if not go:
            self.mode = "HOLD"
            self._launched = False   # re-arm the launch for the next start
            if config.CONTROL_MODE == "ACRO":
                # Explicit throttle-at-minimum: the FPV arming handshake.
                # This is what the sim's "put throttle down" guard wants.
                self._send_body_velocity(0, 0, 0, 0, "pre-GO: thrust 0 (arm handshake)",
                                         grounded=True)
            else:
                # velocity backend: zero-velocity setpoints read as mid-throttle,
                # so stay fully silent pre-GO; log intent only
                self.state.set_last_cmd({"t": time.time(), "mode": "HOLD",
                                         "vx": 0, "vy": 0, "vz": 0, "yaw_rate": 0,
                                         "why": "pre-GO: silent (arming guard)"})
            return
        if race is not None and race.finished:
            self._send_body_velocity(0, 0, 0, 0, "race FINISHED")
            return

        # Detect gate passage via active_gate_index change.
        if race is not None:
            if self._last_gate_idx is not None and race.active_gate_index > self._last_gate_idx:
                # PASSED A GATE. Carry straight through, wipe memory of the
                # old gate so we can't re-target it, then hunt the next one.
                self._commit_until = 0.0
                self._carry_until = time.time() + config.CARRY_TIME_S
                self._last_seen = None
                self._pingpong_events = []
                self.mode = "CARRY"
                print(f">> GATE {self._last_gate_idx} PASSED! carrying through, "
                      f"next: gate {race.active_gate_index}", flush=True)
            self._last_gate_idx = race.active_gate_index

        if time.time() < getattr(self, "_carry_until", 0.0):
            self.mode = "CARRY"
            self._prev_mode = "CARRY"
            self._send_body_velocity(config.COMMIT_SPEED, 0, 0, 0, "carrying past passed gate")
            return

        now = time.time()

        # ------------------------------------------------------------ LAUNCH
        # The drone starts ON THE GROUND. Take off before doing anything else.
        if not self._launched:
            self._launched = True
            self._launch_until = now + config.LAUNCH_TIME_S
            self._race_go_t = now
        if not self._calib_done and not self._calib_plan and config.CONTROL_MODE == "ACRO":
            # schedule pulses to run DURING the launch climb, after 0.4s airborne
            t0 = now + 0.4
            step = config.CALIB_PULSE_S + config.CALIB_SETTLE_S
            self._calib_plan = [
                ("pitch", t0, t0 + config.CALIB_PULSE_S),
                ("roll", t0 + step, t0 + step + config.CALIB_PULSE_S),
                ("yaw", t0 + 2 * step, t0 + 2 * step + config.CALIB_PULSE_S),
            ]
            self._launch_until = max(self._launch_until, t0 + 3 * step + 0.1)

        if now < self._launch_until:
            self.mode = "LAUNCH"
            # boost only briefly for liftoff; calibration pulses run at hover
            # thrust so the launch doesn't turn into a trip to the ceiling
            boosting = (now - (self._launch_until - self._launch_total())) < 0.4
            base = (config.PLAYBOOK[0][5] if config.PLAYBOOK and len(config.PLAYBOOK[0]) > 5
                    else self._hover_trim)
            thrust = clamp(base +
                           (config.LAUNCH_THRUST_BOOST if boosting else 0.0),
                           config.THRUST_MIN, config.THRUST_MAX)
            rr = pr = yr = 0.0
            why = "launching off the ground"
            if boosting and time.time() - snap["flow_time"] < 0.5:
                self._launch_flow.append(snap["flow_dy"])
            if not self._calib_done:
                gyro = snap["imu_gyro"] or (0.0, 0.0, 0.0)
                for axis, ts, te in self._calib_plan:
                    if ts <= now < te:
                        pulse = config.CALIB_PULSE_RATE
                        if axis == "pitch":
                            pr = pulse
                            self._calib_readings.setdefault("pitch", []).append(gyro[1])
                        elif axis == "roll":
                            rr = pulse
                            self._calib_readings.setdefault("roll", []).append(gyro[0])
                        elif axis == "yaw":
                            yr = pulse
                            self._calib_readings.setdefault("yaw", []).append(gyro[2])
                        why = f"launch + calib pulse: {axis}"
                        break
                if self._calib_plan and now >= self._calib_plan[-1][2]:
                    self._finish_calibration()
            self.send_attitude_rates(rr, pr, yr, thrust)
            self.state.set_last_cmd({"t": now, "mode": "LAUNCH", "vx": 0, "vy": 0,
                                     "vz": 0, "yaw_rate": yr,
                                     "why": f"{why} (thrust {thrust:.2f})"})
            return

        # ----------------------------------------------------- anti-grind
        # If collisions are racking up fast, we're scraping a surface:
        # override everything and climb away from it.
        if now < self._unstick_until:
            self.mode = "UNSTICK"
            self._send_body_velocity(0, 0, -config.UNSTICK_CLIMB, 0, "UNSTICK: climbing off surface")
            return
        if now - self._col_ref_t >= 0.5:
            rate = (snap["n_collisions"] - self._col_ref_n) / max(0.001, now - self._col_ref_t)
            self._col_ref_t = now
            self._col_ref_n = snap["n_collisions"]
            if rate > config.UNSTICK_COLLISION_RATE:
                self._unstick_until = now + config.UNSTICK_TIME_S
                self.mode = "UNSTICK"
                self._send_body_velocity(0, 0, -config.UNSTICK_CLIMB, 0,
                                         f"UNSTICK: {rate:.0f} hits/sec, climbing")
                return

        self._last_flow_dy = snap.get("flow_dy") or 0.0
        det = snap["detection"]
        det_fresh = (det is not None and
                     time.time() - snap["detection_time"] < config.DETECT_STALE_S)
        # ---- PLAYBOOK: scripted opening on a course that never changes.
        # Runs once from GO; vision takes over the moment a real gate is
        # close (big) ahead, or when the script is exhausted.
        # NEW RACE detected -> reset the playbook (a lingering python
        # process was carrying an exhausted script into runs 2, 3, ...,
        # silently reverting them to pure-vision behavior)
        _rs0 = snap.get("race")
        _rid = getattr(_rs0, "race_start_boot_time_ms", None) if _rs0 else None
        if _rid is not None and _rid != getattr(self, "_pb_race_id", None):
            self._pb_race_id = _rid
            self._pb_idx = 0
            self._pb_gate_idx = 0
            self._pb_pin = None
            self._race_go_t = None
            self._launched = False
            if hasattr(self, "_pb_step_t0"):
                del self._pb_step_t0
            print(">> new race detected: playbook reset to step 1")
        # gate-pass event advances the playbook to the next pace note
        _rs = snap.get("race")
        _gidx = getattr(_rs, "active_gate_index", 0) if _rs else 0
        if _gidx != getattr(self, "_pb_gate_idx", 0):
            self._pb_gate_idx = _gidx
            if getattr(self, "_pb_idx", 0) < len(config.PLAYBOOK):
                self._pb_idx = getattr(self, "_pb_idx", 0) + 1
                self._pb_step_t0 = time.time()
                print(f">> PLAYBOOK advanced by gate pass -> step {self._pb_idx + 1}")
        if getattr(self, "_pb_idx", 0) < len(config.PLAYBOOK) and self.mode != "COMMIT":
            big_gate = det_fresh and det.w >= config.PLAYBOOK_GATE_W
            if big_gate:
                self._pb_idx = len(config.PLAYBOOK)   # vision takes over for good
            else:
                now_pb = time.time()
                if not hasattr(self, "_pb_step_t0"):
                    self._pb_step_t0 = now_pb
                step = config.PLAYBOOK[getattr(self, "_pb_idx", 0)]
                if now_pb - self._pb_step_t0 >= step[0]:
                    self._pb_idx = getattr(self, "_pb_idx", 0) + 1
                    self._pb_step_t0 = now_pb
                if self._pb_idx < len(config.PLAYBOOK):
                    step = config.PLAYBOOK[self._pb_idx]
                    dur, pvx, pvy, pvz, pyr = step[:5]
                    self._pb_pin = step[5] if len(step) > 5 else None
                    self._send_body_velocity(pvx, pvy, pvz, pyr,
                        f"PLAYBOOK step {self._pb_idx + 1}/{len(config.PLAYBOOK)} pin={self._pb_pin}")
                    return
                self._pb_pin = None
        # TUBE-PRIMARY: while the blue line is visible, tiny far-away gate
        # blips must NOT hijack control -- that is the wrong-gate, angled
        # approach failure. A gate only takes over when genuinely close.
        tube0 = snap.get("tube")
        tube0_fresh = (config.TUBE_ENABLE and tube0 is not None and tube0.visible
                       and time.time() - snap.get("tube_time", 0.0) < 0.5)
        if (det_fresh and tube0_fresh and self.mode != "COMMIT"
                and det.w < config.TUBE_GATE_TAKEOVER_W):
            det_fresh = False   # ignore the blip; ride the line

        # ----------------------------------------------------------- COMMIT
        if self.mode == "COMMIT":
            if time.time() < self._commit_until:
                # keep steering GENTLY at the gate while punching through --
                # frozen-straight commits were missing by inches
                yr, vzc, vyc = 0.0, 0.0, 0.0
                if det_fresh:
                    caz, cel = self._angles(det)
                    yr = clamp(config.YAW_KP * caz * self._steer_yaw_sign *
                               config.COMMIT_STEER_GAIN,
                               -config.MAX_YAW_RATE, config.MAX_YAW_RATE)
                    vzc = clamp(-config.EL_VZ_KP * cel * config.COMMIT_STEER_GAIN,
                                -config.MAX_VZ, config.MAX_VZ)
                    if config.STRAFE_ENABLE:
                        vyc = clamp(config.STRAFE_VY_KP * caz * config.STRAFE_SIGN,
                                    -config.STRAFE_VY_MAX, config.STRAFE_VY_MAX)
                self._send_body_velocity(config.COMMIT_SPEED, vyc, vzc, yr,
                                         "committed through gate (steering)")
                return
            self.mode = "SEARCH"

        # ----------------------------------------------------------- SEARCH
        if not det_fresh:
            self._el_bias *= max(0.0, 1.0 - config.EL_BIAS_DECAY / config.CONTROL_HZ)
            if self._prev_mode == "TRACK":
                self._pingpong_check(time.time())
            self.mode = "SEARCH"
            self._prev_mode = "SEARCH"
            now_s = time.time()
            # ---- TUBE FOLLOW: the blue guidance line threads every gate,
            # in order, head-on. If we can see it, follow it instead of
            # blind-scanning: this both finds the RIGHT next gate and sets
            # up perpendicular (head-on) approaches.
            tube = snap.get("tube")
            tube_fresh = (config.TUBE_ENABLE and tube is not None and
                          tube.visible and
                          now_s - snap.get("tube_time", 0.0) < 0.5)
            if tube_fresh:
                yr = clamp(config.TUBE_YAW_KP * tube.far_x * self._steer_yaw_sign,
                           -config.MAX_YAW_RATE, config.MAX_YAW_RATE)
                vyt = clamp(config.TUBE_VY_KP * tube.near_x * config.STRAFE_SIGN,
                            -config.STRAFE_VY_MAX, config.STRAFE_VY_MAX)
                self._flow_motion_check(now_s, snap, 0.0)
                self._send_body_velocity(config.TUBE_SPEED, vyt, 0.0, yr,
                                         f"TUBE follow: near={tube.near_x:+.2f} far={tube.far_x:+.2f}")
                return
            yaw_rate = self._committed_scan_dir(now_s, 0.0) * config.SEARCH_YAW_RATE
            vz = 0.0
            hint = "no memory"
            if self._last_seen and now_s - self._last_seen_t < config.SEARCH_MEMORY_S:
                nx, ny = self._last_seen
                if ny > 0.4:
                    vz = +config.SEARCH_VZ                 # was low in frame: descend
                    yaw_rate = 0.0                         # HOLD HEADING while moving vertically
                    hint = "last seen below: descending, holding heading"
                elif ny < -0.4:
                    vz = -config.SEARCH_VZ                 # was high in frame: climb
                    yaw_rate = 0.0
                    hint = "last seen above: climbing, holding heading"
                else:
                    pref = (1.0 if nx >= 0 else -1.0) * self._steer_yaw_sign
                    yaw_rate = self._committed_scan_dir(now_s, pref) * config.SEARCH_YAW_RATE
                    hint = f"last seen to the side: committed scan {yaw_rate:+.1f}"
            else:
                if self._nomem_since is None:
                    self._nomem_since = now_s
                cycle = config.SEARCH_SWEEP_DOWN_S + config.SEARCH_SWEEP_UP_S
                t_in = (now_s - self._nomem_since) % cycle
                vdir = +1.0 if t_in < config.SEARCH_SWEEP_DOWN_S else -1.0  # mostly down: gates are low
                vz = vdir * config.SEARCH_DESCEND
                hint = f"no memory: altitude sweep {'down' if vdir > 0 else 'up'}"
            self._flow_motion_check(now_s, snap, vz)
            self._hover_drift_check(now_s, snap, vz)
            self._send_body_velocity(0, 0, vz, yaw_rate,
                                     f"searching ({hint}) steer={self._steer_yaw_sign:+.0f}")
            return

        # ------------------------------------------------------------ TRACK
        self._nomem_since = None
        # optical-flow motion check
        self._flow_hist = []         # (t, |dy|, vz_intent)
        self._flow_nudges = 0
        self._last_flow_nudge = 0.0
        self._flow_last_vz_sign = 0
        # signed flow: learned during launch boost (we KNOW we're climbing)
        self._flow_up_sign = 0.0     # 0 = unknown / untrusted
        self._flow_sign_votes = []   # recent candidate up-signs
        self._launch_flow = []
        self._drift_hist = []        # (t, dy) while commanding hover
        self._last_drift_nudge = 0.0
        self._drift_nudges = 0
        self._drift_last_mag = None
        if self._prev_mode != "TRACK":
            self._track_since = time.time()
        self.mode = "TRACK"
        self._prev_mode = "TRACK"

        cx0 = det.frame_w / 2.0
        cy0 = det.frame_h / 2.0
        az, el_body = self._angles(det)

        # remember where we saw it (normalized, + = right / below)
        self._last_seen = ((det.cx - cx0) / cx0, (det.cy - cy0) / cy0)
        self._last_seen_t = time.time()
        self._last_track_az = az

        # Trigger COMMIT only when the gate fills the frame AND is centered.
        wide = det.w / det.frame_w >= config.COMMIT_WIDTH_FRAC
        # the closer we are, the more a small angular error misses by:
        # tighten the el gate when the gate fills most of the frame
        el_max = config.COMMIT_EL_MAX * (0.7 if det.area_frac > 0.30 else 1.0)
        centered = abs(az) <= config.COMMIT_AZ_MAX and abs(el_body) <= el_max
        if wide and centered:
            self.mode = "COMMIT"
            self._commit_until = time.time() + config.COMMIT_TIME_S
            self._send_body_velocity(config.COMMIT_SPEED, 0, 0, 0, "commit: close + centered")
            return

        # Yaw toward the gate; slow down when badly misaligned.
        # _steer_yaw_sign is camera-verified: flips itself if the gate drifts
        # away while being steered at (the "ping-pong" failure).
        self._yaw_steer_check(time.time(), az)
        yaw_kp = config.YAW_KP * (config.CLOSE_YAW_BOOST
                                  if det.area_frac > config.CLOSE_AREA else 1.0)
        yaw_rate = clamp(yaw_kp * az * self._steer_yaw_sign,
                         -config.MAX_YAW_RATE, config.MAX_YAW_RATE)
        align = clamp(1.0 - abs(az) / config.ALIGN_FALLOFF_RAD, 0.0, 1.0)
        # climb-first: vertical misalignment throttles forward speed so the
        # forward pitch never ejects a high/low gate from the camera's view
        vfac = clamp(1.0 - abs(el_body) / config.EL_SLOW_RAD, 0.0, 1.0)
        vx = max(config.VX_EL_MIN, config.MAX_FWD * align * vfac)
        # ALTITUDE-FIRST (hysteresis): badly above/below the gate line ->
        # STOP advancing entirely and fix altitude in place. Flying forward
        # while under a gate is what produced every under-pass and backwards
        # entry so far: the pursuit curve always arrives BELOW the opening.
        if abs(el_body) > config.EL_HOLD_RAD:
            self._alt_hold = True
        elif abs(el_body) < config.EL_ADVANCE_RAD:
            self._alt_hold = False
        if getattr(self, "_alt_hold", False):
            vx = 0.0
        # ANTI-ORBIT BRAKE: gate is close but well off to the side = we are
        # sliding past it. Pitch BACK to dump the momentum instead of
        # carving a wide circle around the gate (yaw can't kill drift).
        braking = (abs(az) > config.BRAKE_AZ_RAD and
                   det.area_frac > config.BRAKE_AREA_MIN)
        if braking:
            vx = config.BRAKE_VX
        # STRAFE: bank toward the gate so lateral error closes by moving
        # sideways, not just by rotating. steer sign shared with yaw.
        vy = 0.0
        if config.STRAFE_ENABLE:
            vy = clamp(config.STRAFE_VY_KP * az * config.STRAFE_SIGN,
                       -config.STRAFE_VY_MAX, config.STRAFE_VY_MAX)
        if wide and not centered:
            vx = min(vx, config.APPROACH_VX)   # at the gate mouth: line up first
        if (det.h > config.OBLIQUE_HW_RATIO * det.w and
                det.area_frac > config.OBLIQUE_AREA_MIN):
            vx = min(vx, config.OBLIQUE_VX)    # bad angle: don't rush the miss
        self._forward_check(time.time(), det, vx)

        # Climb/descend proportional to elevation error -- deliberately
        # decoupled from vx so a high gate still gets a decisive climb even
        # while climb-first is holding forward speed near zero.
        vz = clamp(-config.EL_VZ_KP * el_body, -config.MAX_VZ, config.MAX_VZ)
        recent_w = [w for (t, w) in self._w_hist if time.time() - t <= 1.0]
        w_growing = len(recent_w) >= 3 and det.w >= recent_w[0] * 1.08
        self._vertical_check(time.time(), el_body, vz, det.w, w_growing)

        # CAMERA-TRUTH THRUST INTEGRATOR: as long as the gate stays above
        # (below) image center, thrust bias ratchets up (down) until the
        # image itself says we're rising (sinking) toward it. Immune to
        # accel-estimate leaks and wrong hover guesses.
        now_el = time.time()
        if self._el_last_t is not None:
            dt_el = min(0.1, now_el - self._el_last_t)
            self._el_bias = clamp(self._el_bias + config.EL_KI * el_body * dt_el,
                                  -config.EL_BIAS_MAX, config.EL_BIAS_MAX)
        self._el_last_t = now_el

        self._send_body_velocity(vx, vy, vz, yaw_rate,
                                 f"track: az={math.degrees(az):+.0f}deg el={math.degrees(el_body):+.0f}deg "
                                 f"vz={vz:+.1f} vy={vy:+.1f}"
                                 f"{' BRAKE' if braking else ''}"
                                 f"{' ALT-HOLD' if getattr(self, '_alt_hold', False) else ''} "
                                 f"elbias={self._el_bias:+.2f}")

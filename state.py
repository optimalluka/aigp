"""
Thread-safe shared state.

Every receiver thread (mavlink, vision) WRITES here.
The controller and logger READ consistent snapshots from here.
This replaces the template's empty `shared_data = {}` dict with something
that actually holds the data and can't be corrupted across threads.
"""

import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field


@dataclass
class Gate:
    gate_id: int
    x: float
    y: float
    z: float
    qw: float
    qx: float
    qy: float
    qz: float
    width: float
    height: float


@dataclass
class Odometry:
    t_us: int = 0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0


@dataclass
class RaceStatus:
    sim_boot_time_ms: int = 0
    race_start_boot_time_ms: int = -1
    race_finish_time_ns: int = -1
    active_gate_index: int = 0
    last_gate_race_time: int = -1

    @property
    def started(self) -> bool:
        # The sim appears to set race_start_boot_time_ms when the COUNTDOWN
        # begins (pointing at the future go moment). Only treat the race as
        # started once sim time has actually reached it -- launching earlier
        # is a false start ("too soon").
        if self.race_start_boot_time_ms is None or self.race_start_boot_time_ms <= 0:
            return False
        return self.sim_boot_time_ms >= self.race_start_boot_time_ms

    @property
    def finished(self) -> bool:
        return self.race_finish_time_ns is not None and self.race_finish_time_ns > 0


class State:
    def __init__(self):
        self._lock = threading.Lock()

        self.odom = None            # Odometry | None
        self.attitude = None        # (roll, pitch, yaw, t_ms) | None
        self.race = None            # RaceStatus | None
        self.gates = []             # list[Gate]
        self.armed = False
        self.collisions = []        # list of (wall_time, collision_id, threat, impulse)

        # vision stats
        self.frame_count = 0
        self.last_frame_wall_time = 0.0

        # diagnostics: msg_type -> count of everything received
        self.msg_counts = defaultdict(int)

        # latest gate detection from vision (gate_vision.GateDetection)
        self.detection = None
        self.detection_time = 0.0

        # latest motor outputs from ACTUATOR_OUTPUT_STATUS (list of 4 floats)
        self.motors = None
        self.motors_time = 0.0

        # IMU + estimated attitude (from attitude.AttitudeEstimator)
        self.imu_accel = None       # (ax, ay, az) m/s^2
        self.imu_gyro = None        # (gx, gy, gz) rad/s
        self.est_roll = 0.0         # rad
        self.est_pitch = 0.0        # rad
        self.est_vz = 0.0           # m/s, + = descending

        # global vertical optical flow (px/frame at reduced res)
        self.flow_dy = 0.0
        self.flow_time = 0.0

        # what the controller last commanded (for logging)
        self.last_cmd = None        # dict | None

    # ------------------------------------------------------------------ writes
    def set_odom(self, odom: Odometry):
        with self._lock:
            self.odom = odom

    def set_attitude(self, roll, pitch, yaw, t_ms):
        with self._lock:
            self.attitude = (roll, pitch, yaw, t_ms)

    def set_race(self, race: RaceStatus):
        with self._lock:
            self.race = race

    def set_gates(self, gates):
        with self._lock:
            self.gates = gates

    def set_armed(self, armed: bool):
        with self._lock:
            self.armed = armed

    def add_collision(self, collision_id, threat, impulse):
        with self._lock:
            self.collisions.append((time.time(), collision_id, threat, impulse))

    def count_msg(self, msg_type: str):
        with self._lock:
            self.msg_counts[msg_type] += 1

    def count_frame(self):
        with self._lock:
            self.frame_count += 1
            self.last_frame_wall_time = time.time()

    def set_imu(self, accel, gyro, est_roll, est_pitch, est_vz=0.0):
        with self._lock:
            self.imu_accel = accel
            self.imu_gyro = gyro
            self.est_roll = est_roll
            self.est_pitch = est_pitch
            self.est_vz = est_vz

    def set_flow(self, dy):
        with self._lock:
            self.flow_dy = dy
            self.flow_time = time.time()

    def set_motors(self, motors):
        with self._lock:
            self.motors = motors
            self.motors_time = time.time()

    def set_detection(self, det):
        with self._lock:
            self.detection = det
            self.detection_time = time.time()

    def set_tube(self, tube):
        with self._lock:
            self.tube = tube
            self.tube_time = time.time()

    def set_last_cmd(self, cmd: dict):
        with self._lock:
            self.last_cmd = cmd

    # ------------------------------------------------------------------- reads
    def snapshot(self):
        """Consistent copy of everything, for controller + logger."""
        with self._lock:
            return {
                "odom": self.odom,
                "attitude": self.attitude,
                "race": self.race,
                "gates": list(self.gates),
                "armed": self.armed,
                "n_collisions": len(self.collisions),
                "last_collision": self.collisions[-1] if self.collisions else None,
                "frame_count": self.frame_count,
                "last_frame_wall_time": self.last_frame_wall_time,
                "msg_counts": dict(self.msg_counts),
                "detection": self.detection,
                "tube": getattr(self, "tube", None),
                "tube_time": getattr(self, "tube_time", 0.0),
                "detection_time": self.detection_time,
                "motors": list(self.motors) if self.motors else None,
                "imu_accel": self.imu_accel,
                "imu_gyro": self.imu_gyro,
                "est_roll": self.est_roll,
                "est_pitch": self.est_pitch,
                "est_vz": self.est_vz,
                "flow_dy": self.flow_dy,
                "flow_time": self.flow_time,
                "last_cmd": self.last_cmd,
            }

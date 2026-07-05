"""
Attitude estimator: complementary filter on HIGHRES_IMU.

In ACRO mode the sim gives no attitude -- we must estimate roll/pitch
ourselves from the gyro (fast, drifts) corrected by the accelerometer
(slow, gravity reference). Yaw is not estimated (drifts, and we don't
need absolute yaw -- vision handles heading).

Axis sign conventions differ between sims; the --nudge probe prints the
raw at-rest accelerometer vector so the conventions can be verified and
the EST_*_SIGN knobs in config.py flipped if needed.
"""

import math

import config


class AttitudeEstimator:
    def __init__(self):
        self.roll = 0.0     # rad, + = right wing down (FRD convention)
        self.pitch = 0.0    # rad, + = nose up
        self.last_t_us = None
        self.accel = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.vz_down = 0.0   # m/s, + = descending (leaky integral of vertical accel)

    def update(self, t_us, ax, ay, az, gx, gy, gz):
        self.accel = (ax, ay, az)
        self.gyro = (gx, gy, gz)

        if self.last_t_us is None:
            self.last_t_us = t_us
            return
        dt = (t_us - self.last_t_us) / 1e6
        self.last_t_us = t_us
        if dt <= 0 or dt > 0.5:
            return

        # vertical velocity: rotate specific force into world-down, remove g,
        # integrate with a leak so drift stays bounded
        cph, sph = math.cos(self.roll), math.sin(self.roll)
        cth, sth = math.cos(self.pitch), math.sin(self.pitch)
        f_down = -sth * ax + sph * cth * ay + cph * cth * az
        a_down = f_down + 9.81
        self.vz_down += a_down * dt
        self.vz_down *= max(0.0, 1.0 - config.VZ_EST_LEAK * dt)

        # gyro integration (small-angle body rates -> euler rates)
        self.roll += config.EST_GYRO_ROLL_SIGN * gx * dt
        self.pitch += config.EST_GYRO_PITCH_SIGN * gy * dt

        # accelerometer gravity reference (only trust when |a| ~ g)
        a_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if 0.5 * 9.81 < a_norm < 1.5 * 9.81:
            roll_acc = config.EST_ACC_ROLL_SIGN * math.atan2(ay, -az if az < 0 else az + 1e-9)
            pitch_acc = config.EST_ACC_PITCH_SIGN * math.atan2(-ax, math.sqrt(ay * ay + az * az) + 1e-9)
            alpha = config.EST_ACC_ALPHA
            self.roll = (1 - alpha) * self.roll + alpha * roll_acc
            self.pitch = (1 - alpha) * self.pitch + alpha * pitch_acc

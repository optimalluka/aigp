"""
Housekeeping loop: timesync requests + client heartbeat.

The technical spec (VADR-TS-003, sections 4.4 and 5.2) requires the CLIENT
to maintain heartbeat messages at a minimum rate of 2 Hz. Neither the
official template nor our first version sent any -- this loop fixes that.
We send at 10 Hz alongside timesync, comfortably above the minimum.
"""

import threading
import time

from pymavlink import mavutil

TIMESYNC_REQUEST_HZ = 10


class TimeSync:
    def __init__(self, mavlink_connection):
        self.conn = mavlink_connection
        self.is_running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.is_running = False

    def _loop(self):
        while self.is_running:
            try:
                now = int(time.time_ns())
                self.conn.mav.timesync_send(now, 0)
                self.conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,  # base_mode
                    0,  # custom_mode
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
            except (ConnectionResetError, OSError):
                pass
            time.sleep(1.0 / TIMESYNC_REQUEST_HZ)

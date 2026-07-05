"""
MAVLink receiver.

Transport/parsing logic is taken from the official PyAIPilotExample template
(message IDs, struct formats, track-chunk reassembly) -- that part is the
competition's interface spec and we don't reinvent it.

Difference from the template: every handler actually STORES what it parses
into the shared State object instead of throwing it away.
"""

import struct
import threading
import time

from pymavlink import mavutil

from attitude import AttitudeEstimator
from state import State, Odometry, RaceStatus, Gate

ENCAPSULATED_RACE_STATUS_MSG_ID = 1
ENCAPSULATED_TRACK_INFO_MSG_ID = 2


class MAVLinkRX:
    def __init__(self, mavlink_connection, state: State):
        self.conn = mavlink_connection
        self.state = state
        self.is_running = False
        self.thread = None

        self.track_chunks = {}
        self.expected_num_track_chunks = {}
        self.estimator = AttitudeEstimator()

    @classmethod
    def start(cls, mavlink_connection, state: State):
        rx = cls(mavlink_connection, state)
        rx.is_running = True
        rx.thread = threading.Thread(target=rx._loop, daemon=True)
        rx.thread.start()
        return rx

    def stop(self):
        self.is_running = False

    # ------------------------------------------------------------------ loop
    def _loop(self):
        while self.is_running:
            try:
                msg = self.conn.recv_match(blocking=False)
            except (ConnectionResetError, OSError):
                # Windows raises this when the sim closes its UDP port
                # (e.g. between courses). Stay alive; it comes back.
                time.sleep(0.5)
                continue

            if msg is None:
                time.sleep(0.001)
                continue

            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                continue

            self.state.count_msg(msg_type)

            if msg_type == "HEARTBEAT":
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.state.set_armed(armed)

            elif msg_type == "ATTITUDE":
                self.state.set_attitude(msg.roll, msg.pitch, msg.yaw, msg.time_boot_ms)

            elif msg_type == "ODOMETRY":
                self.state.set_odom(Odometry(
                    t_us=msg.time_usec,
                    x=msg.x, y=msg.y, z=msg.z,
                    qw=msg.q[0], qx=msg.q[1], qy=msg.q[2], qz=msg.q[3],
                    vx=msg.vx, vy=msg.vy, vz=msg.vz,
                ))

            elif msg_type == "LOCAL_POSITION_NED":
                # Only use as odometry source if ODOMETRY never arrives;
                # keep it simple: store it too (no quaternion -> keep previous).
                prev = self.state.snapshot()["odom"]
                q = (prev.qw, prev.qx, prev.qy, prev.qz) if prev else (1.0, 0.0, 0.0, 0.0)
                self.state.set_odom(Odometry(
                    t_us=msg.time_boot_ms * 1000,
                    x=msg.x, y=msg.y, z=msg.z,
                    qw=q[0], qx=q[1], qy=q[2], qz=q[3],
                    vx=msg.vx, vy=msg.vy, vz=msg.vz,
                ))

            elif msg_type == "HIGHRES_IMU":
                self.estimator.update(msg.time_usec,
                                      msg.xacc, msg.yacc, msg.zacc,
                                      msg.xgyro, msg.ygyro, msg.zgyro)
                self.state.set_imu(self.estimator.accel, self.estimator.gyro,
                                   self.estimator.roll, self.estimator.pitch,
                                   self.estimator.vz_down)

            elif msg_type == "ACTUATOR_OUTPUT_STATUS":
                self.state.set_motors([msg.actuator[0], msg.actuator[1],
                                       msg.actuator[2], msg.actuator[3]])

            elif msg_type == "COLLISION":
                # 1001 = gate, 1002 = environment
                self.state.add_collision(msg.id, msg.threat_level, msg.horizontal_minimum_delta)
                kind = {1001: "GATE", 1002: "ENVIRONMENT"}.get(msg.id, str(msg.id))
                print(f"!! COLLISION with {kind} (threat {msg.threat_level})")

            elif msg_type == "ENCAPSULATED_DATA":
                self._on_encapsulated(msg)

            elif msg_type == "DATA_TRANSMISSION_HANDSHAKE":
                transfer_id = msg.width
                self.track_chunks[transfer_id] = {}
                self.expected_num_track_chunks[transfer_id] = msg.packets

    # ------------------------------------------------------- encapsulated data
    def _on_encapsulated(self, msg):
        raw = bytes(msg.data)
        data_type = raw[0]

        if int(data_type) == ENCAPSULATED_RACE_STATUS_MSG_ID:
            (data_type, sim_boot_time_ms, race_start_boot_time_ms,
             race_finish_time_ns, active_gate_index, last_gate_race_time) = struct.unpack_from("<BQqqIq", raw)
            self.state.set_race(RaceStatus(
                sim_boot_time_ms=sim_boot_time_ms,
                race_start_boot_time_ms=race_start_boot_time_ms,
                race_finish_time_ns=race_finish_time_ns,
                active_gate_index=active_gate_index,
                last_gate_race_time=last_gate_race_time,
            ))

        elif int(data_type) == ENCAPSULATED_TRACK_INFO_MSG_ID:
            data_type, transfer_id = struct.unpack_from("<BH", raw)
            if transfer_id not in self.expected_num_track_chunks:
                return
            self.track_chunks[transfer_id][msg.seqnr] = raw[3:]
            if len(self.track_chunks[transfer_id]) == self.expected_num_track_chunks[transfer_id]:
                full = bytes()
                for i in range(len(self.track_chunks[transfer_id])):
                    full += self.track_chunks[transfer_id][i]
                del self.track_chunks[transfer_id]
                del self.expected_num_track_chunks[transfer_id]
                self._on_track_data(full)

    def _on_track_data(self, payload):
        num_gates, = struct.unpack_from("<H", payload)
        payload = payload[2:]
        gates = []
        for _ in range(num_gates):
            (gate_id, px, py, pz, qw, qx, qy, qz, width, height) = struct.unpack_from("<Hfffffffff", payload)
            payload = payload[38:]
            gates.append(Gate(gate_id, px, py, pz, qw, qx, qy, qz, width, height))
        gates.sort(key=lambda g: g.gate_id)
        self.state.set_gates(gates)
        print(f">> TRACK DATA received: {num_gates} gates")
        for g in gates:
            print(f"   gate {g.gate_id}: pos=({g.x:.1f}, {g.y:.1f}, {g.z:.1f})  {g.width:.1f}x{g.height:.1f} m")

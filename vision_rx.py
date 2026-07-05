"""
Vision receiver.

UDP chunk-reassembly transport is from the official template.
FlightSim 2.0 blocks odometry/track data, so vision IS the navigation now.
Every frame runs the gate detector and publishes the result to shared
state for the controller. Periodically saves ANNOTATED frames (with the
detection overlay) to logs/<run>/frames/ so we can replay what it saw.
"""

import os
import socket
import struct
import threading
import time

import cv2
import numpy as np

import config
import gate_vision
from state import State


class VisionRX:
    def __init__(self, state: State, run_dir: str):
        self.state = state
        self.is_running = True
        self.frames_dir = os.path.join(run_dir, "frames")
        if config.SAVE_FRAME_EVERY_S > 0:
            os.makedirs(self.frames_dir, exist_ok=True)
        self._last_save = 0.0
        self._prev_small = None
        self._lock_pos = None
        self._lock_t = 0.0
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.is_running = False

    def _loop(self):
        header_format = "<IHHIIQ"
        header_sz = struct.calcsize(header_format)
        frames = {}

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.5)
        sock.bind((config.VISION_UDP_IP, config.VISION_UDP_PORT))
        print("Vision: listening for camera frames...")

        while self.is_running:
            try:
                packet, _addr = sock.recvfrom(65536)
            except socket.timeout:
                continue

            header = packet[:header_sz]
            payload = packet[header_sz:]
            (frame_id, chunk_id, total_chunks, jpeg_size,
             payload_size, sim_time_ns) = struct.unpack(header_format, header)

            if frame_id not in frames:
                frames[frame_id] = {"chunks": {}, "total": total_chunks}
            frames[frame_id]["chunks"][chunk_id] = payload

            if len(frames[frame_id]["chunks"]) == total_chunks:
                jpeg_bytes = bytearray()
                complete = True
                for i in range(total_chunks):
                    if i not in frames[frame_id]["chunks"]:
                        complete = False
                        break
                    jpeg_bytes.extend(frames[frame_id]["chunks"][i])

                if complete:
                    img = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        self._process(img)
                del frames[frame_id]

                # prune stale partial frames so dict never grows forever
                if len(frames) > 30:
                    for fid in sorted(frames.keys())[:-10]:
                        del frames[fid]

    def _process(self, img):
        self.state.count_frame()

        prefer = None
        if (self._lock_pos is not None and
                time.time() - self._lock_t < config.GATE_LOCK_MEMORY_S):
            prefer = self._lock_pos
        det = gate_vision.detect_gate(img, prefer=prefer)
        try:
            self.state.set_tube(gate_vision.detect_tube(img))
        except Exception:
            pass
        if det is not None:
            self._lock_pos = (det.cx, det.cy)
            self._lock_t = time.time()
        self.state.set_detection(det)   # None is a valid result (= gate lost)

        # global vertical optical flow (cheap phase correlation on a thumbnail)
        small = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (160, 90))
        small = np.float32(small)
        if self._prev_small is not None:
            (dx, dy), _resp = cv2.phaseCorrelate(self._prev_small, small)
            self.state.set_flow(dy)
        self._prev_small = small

        now = time.time()
        if config.SAVE_FRAME_EVERY_S > 0 and now - self._last_save >= config.SAVE_FRAME_EVERY_S:
            self._last_save = now
            cmd = self.state.snapshot()["last_cmd"]
            mode = cmd["mode"] if cmd and "mode" in cmd else ""
            vis = gate_vision.annotate(img, det, mode)
            path = os.path.join(self.frames_dir, f"frame_{int(now * 1000)}.jpg")
            cv2.imwrite(path, vis)

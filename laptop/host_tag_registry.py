"""Multi-tag registration from existing UDP 5008 packets; no survey firmware.
XY estimates assume level flight and the configured camera tilt/offset.
Old firmware only supplies a pose after its own quality gate; no pose is invented.
"""
import math
import socket
import threading
import time
from protocol import parse_at_debug


class HostTagRegistry:
    def __init__(self, drone_id, get_telemetry, callback, port=5008,
                 min_margin=25.0, frames=3, camera_pitch_deg=45.0,
                 camera_forward=0.02, reset_gap=1.0):
        self.drone_id = drone_id
        self.get_telemetry = get_telemetry
        self.callback = callback
        self.port = port
        self.min_margin = min_margin
        self.frames = frames
        self.reset_gap = reset_gap
        self.pitch = math.radians(camera_pitch_deg)
        self.forward = camera_forward
        self.received_at = 0.0
        self.frame_at = 0.0
        self.last_frame = None
        self.hits = {}
        self.last_seen = {}
        self.records = {}
        self.samples = {}
        self.stop_event = threading.Event()
        self.thread = None
        self.sock = None
        self.error = None

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(('', self.port))
        except OSError:
            self.sock.close()
            raise RuntimeError(f'UDP {self.port} unavailable; close tag_debug.py and other tag debug listeners')
        self.sock.settimeout(.2)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2)
        if self.sock:
            self.sock.close()

    def _run(self):
        try:
            while not self.stop_event.is_set():
                try:
                    data, _ = self.sock.recvfrom(4096)
                except socket.timeout:
                    continue
                packet = parse_at_debug(data)
                if packet is not None:
                    self.process(packet, time.monotonic())
        except Exception as exc:
            self.error = str(exc)

    def process(self, packet, now):
        if packet.drone_id != self.drone_id:
            return
        self.received_at = now
        # Firmware repeats the last camera frame at 10 Hz; count each frame once.
        if not packet.frame_ms or packet.frame_ms == self.last_frame:
            return
        self.last_frame = packet.frame_ms
        self.frame_at = now
        seen = set()
        for d in packet.detections:
            if (d.id in seen or not 0 <= d.id <= 21 or d.hamming > 1 or
                    not math.isfinite(d.margin) or d.margin <= self.min_margin):
                continue
            seen.add(d.id)
            if d.id not in self.records and now - self.last_seen.get(d.id, -100) > self.reset_gap:
                self.hits[d.id] = 0
            self.last_seen[d.id] = now
            self.hits[d.id] = self.hits.get(d.id, 0) + 1
            if self.hits[d.id] < self.frames:
                continue
            r = self.records.setdefault(d.id, dict(tag_id=d.id, localized=False,
                    observations=0, x=None, y=None, z=None))
            r['observations'] = self.hits[d.id]
            telem, received = self.get_telemetry()
            if (telem is None or now-received > .5 or packet.proc_ms > 500 or
                    not 0 <= d.pose_err < .5 or d.tz <= 0 or
                    not all(math.isfinite(v) for v in (d.tx,d.ty,d.tz,telem.ned_x,telem.ned_y,telem.heading_rad))):
                continue
            # Old telemetry has yaw but no roll, pitch, altitude or synchronized pose.
            fwd = -math.sin(self.pitch)*d.ty + math.cos(self.pitch)*d.tz + self.forward
            right = d.tx
            c, s = math.cos(telem.heading_rad), math.sin(telem.heading_rad)
            x = telem.ned_x + c*fwd - s*right
            y = telem.ned_y + s*fwd + c*right
            n = self.samples.get(d.id,0)+1
            self.samples[d.id] = n
            r['x'] = x if n==1 else r['x']+(x-r['x'])/n
            r['y'] = y if n==1 else r['y']+(y-r['y'])/n
            r['localized'] = True
        self.callback(self.drone_id, [dict(r) for r in self.records.values()])

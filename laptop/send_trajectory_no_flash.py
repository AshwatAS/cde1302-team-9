# #!/usr/bin/env python3
# """
# Fly a CSV trajectory smoothly on one drone.

# Laptop-only tag registration uses the existing UDP 5008 debug stream.
# No new survey firmware or return command is required.
# The whole trajectory is uploaded first; the ESP32 then plays it on its own
# 20 Hz clock (position + velocity feedforward to PX4), so WiFi latency and
# jitter cannot disturb the flight.

# CSV: columns t,x,y,z (s, NED m); a header row and '#' comments are allowed.
# The trajectory is flown relative to where the drone hovers when it starts.

# Example:
#     python3 laptop/send_trajectory.py --drone-id 2 --takeoff --confirm \
#         --trajectory trajectory/circle_traj.csv
# """

# import argparse
# import csv
# import logging
# import random
# import time
# import threading
# from pathlib import Path

# import numpy as np

# from comms import CommsNode
# from host_tag_registry import HostTagRegistry
# import math
# from protocol import (
#     CMD_HOLD,
#     CMD_LAND,
#     CMD_START,
#     CMD_GOTO,
#     NAV_TRAJ,
#     NAV_ARRIVED,
#     NAV_IDLE,
#     NAV_FLYING,
#     NAV_ROTATING,
#     TRAJ_MAX_PTS,
#     CommandPacket,
#     build_traj_packets,
#     build_traj_start,
# )

# logging.basicConfig(
#     level=logging.INFO,
#     format="%(asctime)s | %(levelname)s | %(message)s",
#     datefmt="%H:%M:%S",
# )
# log = logging.getLogger("send_trajectory_no_flash")

# UPLOAD_GAP_S = 0.025   # ESP drains commands at 10 Hz from a 6-packet UDP queue


# def load_trajectory(path: str, dt: float) -> np.ndarray:
#     """Read t,x,y,z, resample every dt, return offsets from the first point."""
#     data = np.atleast_2d(np.genfromtxt(path, delimiter=",", comments="#"))
#     data = data[~np.isnan(data).any(axis=1)]           # drops the header row
#     if data.shape[0] < 2 or data.shape[1] < 4:
#         raise SystemExit(f"{path}: need at least 2 rows of t,x,y,z")
#     t = data[:, 0]
#     if np.any(np.diff(t) <= 0):
#         raise SystemExit(f"{path}: time column must be strictly increasing")
#     tq = np.arange(t[0], t[-1] + 1e-9, dt)
#     pts = np.column_stack([np.interp(tq, t, data[:, c]) for c in (1, 2, 3)])
#     return pts - pts[0]


# def main() -> None:
#     ap = argparse.ArgumentParser(description="Upload a CSV trajectory to one drone and fly it")
#     ap.add_argument("--drone-id", type=int, required=True)
#     ap.add_argument("--trajectory", required=True, help="CSV with columns t,x,y,z (s, NED m)")
#     ap.add_argument("--config", default=str(Path(__file__).with_name("setup.yaml")),
#                     help="Fleet config (optional)")
#     ap.add_argument("--dt", type=float, default=0.05,
#                     help="Upload sample period (s); 0.05 = ESP nav loop rate")
#     ap.add_argument("--max-speed", type=float, default=0.30,
#                     help="Refuse faster trajectories (m/s); keep below MAV_CMD_SPEED_CAP_MS")
#     ap.add_argument("--takeoff", action="store_true", help="Send CMD_START (arm + take off) first")
#     ap.add_argument("--takeoff-wait", type=float, default=8.0,
#                     help="Seconds from CMD_START to trajectory upload")
#     ap.add_argument("--finish", choices=["land", "hold"], default="land",
#                     help="Action after the trajectory")
#     ap.add_argument("--confirm", action="store_true",
#                     help="Require typing TRAJ-<drone id> before sending any flight commands")
#     ap.add_argument("--tag-port", type=int, default=5008)
#     ap.add_argument("--id-margin", type=float, default=25.0)
#     ap.add_argument("--camera-pitch-deg", type=float, default=45.0)
#     ap.add_argument("--camera-forward", type=float, default=0.02)
#     ap.add_argument("--return-start", action="store_true",
#                     help="After completion, use CMD_GOTO to return to sampled start XY at firmware cruise altitude")
#     ap.add_argument("--return-timeout", type=float, default=120.0)
#     ap.add_argument("--tag-log", default="registered_tags.csv",
#                     help="CSV of registered IDs and estimated local-NED coordinates")
#     ap.add_argument("--telem-port", type=int, default=5005)
#     ap.add_argument("--cmd-port", type=int, default=5006)
#     args = ap.parse_args()
#     if not all(np.isfinite(v) for v in (args.id_margin,args.camera_pitch_deg,args.camera_forward)) or args.id_margin < 0:
#         raise SystemExit("Camera settings and margin must be finite; margin non-negative")
#     if not np.isfinite(args.return_timeout) or args.return_timeout <= 0:
#         raise SystemExit("--return-timeout must be positive and finite")

#     dt_ms = round(args.dt * 1000)
#     if not 1 <= dt_ms <= 1000:
#         raise SystemExit("--dt must be 0.001-1.0 s")
#     pts = load_trajectory(args.trajectory, dt_ms / 1000)
#     duration = (len(pts) - 1) * dt_ms / 1000
#     vmax = np.linalg.norm(np.diff(pts, axis=0), axis=1).max() * 1000 / dt_ms
#     log.info("%s: %d points, %.1f s, max speed %.3f m/s",
#              args.trajectory, len(pts), duration, vmax)
#     if vmax > args.max_speed + 1e-6:
#         raise SystemExit(f"max speed {vmax:.3f} m/s exceeds --max-speed {args.max_speed} m/s")
#     if len(pts) > TRAJ_MAX_PTS:
#         raise SystemExit(f"{len(pts)} points > {TRAJ_MAX_PTS}; use a larger --dt or a shorter trajectory")

#     latest = {}
#     registry = {}
#     registry_lock = threading.Lock()

#     def save_tags():
#         with registry_lock:
#             rows = [dict(drone_id=args.drone_id, **registry[k]) for k in sorted(registry)]
#         path = Path(args.tag_log)
#         path.parent.mkdir(parents=True, exist_ok=True)
#         temp = path.with_suffix(path.suffix + ".tmp")
#         with temp.open("w", newline="") as stream:
#             writer = csv.DictWriter(stream, fieldnames=["drone_id", "tag_id", "localized",
#                                                        "observations", "x", "y", "z"])
#             writer.writeheader()
#             writer.writerows(rows)
#         temp.replace(path)

#     # Create a header-only file immediately, even if no tags are registered.
#     save_tags()
#     log.info("Tag CSV: %s", Path(args.tag_log).resolve())

#     def on_survey(did, entries):
#         if did != args.drone_id:
#             return
#         latest["survey_at"] = time.monotonic()
#         changed = False
#         with registry_lock:
#             for entry in entries:
#                 old = registry.get(entry["tag_id"])
#                 registry[entry["tag_id"]] = entry
#                 if old is None or old["localized"] != entry["localized"]:
#                     changed = True
#                     if entry["localized"]:
#                         log.info("TAG %d REGISTERED ON LAPTOP: estimated telemetry-frame x=%.2f y=%.2f m (level-flight approximation)",
#                                  entry["tag_id"], entry["x"], entry["y"])
#                     else:
#                         log.info("TAG %d REGISTERED: coordinates pending (pose not yet valid)", entry["tag_id"])
#         if changed:
#             save_tags()

#     def on_telemetry(pkt, _ip):
#         if pkt.drone_id == args.drone_id:
#             latest["pkt"] = pkt
#             latest["received_at"] = time.monotonic()

#     def wait_for(cond, timeout_s: float) -> bool:
#         deadline = time.monotonic() + timeout_s
#         while time.monotonic() < deadline:
#             if ("pkt" in latest and time.monotonic() - latest["received_at"] < 3.0
#                     and cond(latest["pkt"])):
#                 return True
#             time.sleep(0.05)
#         return False

#     def send(cmd_type: int, name: str) -> None:
#         if not comms.send_command(args.drone_id, CommandPacket(cmd_type)):
#             raise RuntimeError(f"{name} not sent: drone {args.drone_id} IP unknown")
#         log.info("%s sent", name)

#     comms = CommsNode(
#         listen_port=args.telem_port,
#         cmd_port=args.cmd_port,
#         config_path=args.config if Path(args.config).exists() else None,
#     )
#     comms.on_telemetry(on_telemetry)
#     tags = HostTagRegistry(args.drone_id,
#         lambda: (latest.get("pkt"), latest.get("received_at", 0)), on_survey,
#         port=args.tag_port, min_margin=args.id_margin,
#         camera_pitch_deg=args.camera_pitch_deg, camera_forward=args.camera_forward)
#     tags.start()
#     comms.start()
#     flying = not args.takeoff   # without --takeoff the drone must already hover
#     try:
#         if not wait_for(lambda p: True, 30.0):
#             raise RuntimeError(f"no telemetry from drone {args.drone_id}")

#         if args.confirm:
#             answer = input(f"Type TRAJ-{args.drone_id} to fly {args.trajectory}: ").strip()
#             if answer != f"TRAJ-{args.drone_id}":
#                 log.info("confirmation did not match; trajectory cancelled")
#                 return

#         if not wait_for(lambda p: time.monotonic()-tags.frame_at < 3.0, 5.0):
#             raise RuntimeError("No live AprilTag debug frames on UDP 5008. Check drone IP/network and existing camera debug support; no new survey packets are used.")

#         if args.takeoff:
#             send(CMD_START, "CMD_START")
#             flying = True
#             time.sleep(args.takeoff_wait)

#         # Upload, then start; retry if the drone did not switch to TRAJ.
#         traj_id = random.randint(1, 255)
#         packets = build_traj_packets(traj_id, pts)
#         start = build_traj_start(traj_id, dt_ms)
#         for attempt in range(1, 6):
#             for p in packets:
#                 comms.send_raw(args.drone_id, p)
#                 time.sleep(UPLOAD_GAP_S)
#             if time.monotonic()-latest["received_at"] > 3:
#                 raise RuntimeError("Telemetry stale before trajectory start")
#             origin = (latest["pkt"].ned_x, latest["pkt"].ned_y)
#             comms.send_raw(args.drone_id, start)
#             if wait_for(lambda p: p.nav_state == NAV_TRAJ, 1.5):
#                 break
#             log.warning("attempt %d: trajectory not started (see ESP log)", attempt)
#         else:
#             raise RuntimeError("drone refused the trajectory")

#         log.info("playing trajectory %d (%.1f s)", traj_id, duration)
#         # Firmware may slow playback to 25% while reading rejected tag candidates.
#         end = time.monotonic() + duration / 0.25 + 15.0
#         while True:
#             if tags.error:
#                 raise RuntimeError("Tag monitor failed: " + tags.error)
#             if time.monotonic()-tags.frame_at > 3.0:
#                 raise RuntimeError("Camera debug frames stale during survey")
#             if time.monotonic() - latest["received_at"] > 3.0:
#                 raise RuntimeError("Telemetry stale during trajectory playback")
#             p = latest["pkt"]
#             if p.nav_state == NAV_ARRIVED:
#                 break
#             if p.nav_state != NAV_TRAJ:
#                 raise RuntimeError(f"Trajectory interrupted: {p.nav_state_name}")
#             if time.monotonic() >= end:
#                 raise RuntimeError("Trajectory playback exceeded slow-flight timeout")
#             log.info("pos (%.2f, %.2f)", p.ned_x, p.ned_y)
#             time.sleep(1.0)
#         log.info("trajectory ended, nav state %s", latest["pkt"].nav_state_name)

#         if args.return_start:
#             def return_command():
#                 if not comms.send_command(args.drone_id, CommandPacket(
#                         CMD_GOTO, goal_x=origin[0], goal_y=origin[1])):
#                     raise RuntimeError("Could not send return waypoint")
#             log.info("Returning to sampled start XY (%.2f, %.2f)", *origin)
#             return_command()
#             sent_at = last_sent = time.monotonic()
#             deadline = sent_at + args.return_timeout
#             inside_since = None
#             while True:
#                 now = time.monotonic()
#                 if now-latest["received_at"] > 3:
#                     raise RuntimeError("Telemetry stale during return")
#                 p = latest["pkt"]
#                 distance = math.hypot(p.ned_x-origin[0],p.ned_y-origin[1])
#                 if latest["received_at"] > sent_at and distance <= .25:
#                     if inside_since is None:
#                         inside_since = now
#                     if now-inside_since >= 1.0:
#                         log.info("Returned within 0.25 m of start XY for 1 second")
#                         break
#                 else:
#                     inside_since = None
#                 if now>=deadline:
#                     raise RuntimeError("Return timed out; requesting landing in place")
#                 if p.nav_state not in (NAV_ARRIVED,NAV_IDLE,NAV_FLYING,NAV_ROTATING):
#                     raise RuntimeError(f"Return interrupted: {p.nav_state_name}")
#                 if now-last_sent >= 2 and p.nav_state in (NAV_ARRIVED,NAV_IDLE):
#                     return_command()
#                     last_sent=now
#                 time.sleep(.1)

#         if args.finish == "land":
#             for _ in range(3):   # UDP: repeat LAND
#                 send(CMD_LAND, "CMD_LAND")
#         else:
#             send(CMD_HOLD, "CMD_HOLD")
#     except (Exception, KeyboardInterrupt):
#         if flying:
#             log.error("aborted — sending CMD_LAND")
#             for _ in range(3):
#                 comms.send_command(args.drone_id, CommandPacket(CMD_LAND))
#         raise
#     finally:
#         tags.stop()
#         comms.stop()
#         save_tags()
#         log.info("Saved %d registered tags to %s (telemetry-frame XY; z unavailable)",
#                  len(registry), Path(args.tag_log).resolve())


# if __name__ == "__main__":
#     main()



# #!/usr/bin/env python3
# """
# Fly a CSV trajectory smoothly on one drone.

# Laptop-only tag registration uses the existing UDP 5008 debug stream.
# No new survey firmware or return command is required.
# The whole trajectory is uploaded first; the ESP32 then plays it on its own
# 20 Hz clock (position + velocity feedforward to PX4), so WiFi latency and
# jitter cannot disturb the flight.

# CSV: columns t,x,y,z (s, NED m); a header row and '#' comments are allowed.
# The trajectory is flown relative to where the drone hovers when it starts.

# Example:
#     python3 laptop/send_trajectory.py --drone-id 2 --takeoff --confirm \
#         --trajectory trajectory/circle_traj.csv
# """

# import argparse
# import csv
# import logging
# import random
# import time
# import threading
# from pathlib import Path

# import numpy as np

# from comms import CommsNode
# from host_tag_registry import HostTagRegistry
# import math
# from protocol import (
#     CMD_HOLD,
#     CMD_LAND,
#     CMD_START,
#     CMD_GOTO,
#     NAV_TRAJ,
#     NAV_ARRIVED,
#     NAV_IDLE,
#     NAV_FLYING,
#     NAV_ROTATING,
#     TRAJ_MAX_PTS,
#     CommandPacket,
#     build_traj_packets,
#     build_traj_start,
# )

# logging.basicConfig(
#     level=logging.INFO,
#     format="%(asctime)s | %(levelname)s | %(message)s",
#     datefmt="%H:%M:%S",
# )
# log = logging.getLogger("send_trajectory_no_flash")

# UPLOAD_GAP_S = 0.025   # ESP drains commands at 10 Hz from a 6-packet UDP queue


# def load_trajectory(path: str, dt: float) -> np.ndarray:
#     """Read t,x,y,z, resample every dt, return offsets from the first point."""
#     data = np.atleast_2d(np.genfromtxt(path, delimiter=",", comments="#"))
#     data = data[~np.isnan(data).any(axis=1)]           # drops the header row
#     if data.shape[0] < 2 or data.shape[1] < 4:
#         raise SystemExit(f"{path}: need at least 2 rows of t,x,y,z")
#     t = data[:, 0]
#     if np.any(np.diff(t) <= 0):
#         raise SystemExit(f"{path}: time column must be strictly increasing")
#     tq = np.arange(t[0], t[-1] + 1e-9, dt)
#     pts = np.column_stack([np.interp(tq, t, data[:, c]) for c in (1, 2, 3)])
#     return pts - pts[0]


# def main() -> None:
#     ap = argparse.ArgumentParser(description="Upload a CSV trajectory to one drone and fly it")
#     ap.add_argument("--drone-id", type=int, required=True)
#     ap.add_argument("--trajectory", required=True, help="CSV with columns t,x,y,z (s, NED m)")
#     ap.add_argument("--config", default=str(Path(__file__).with_name("setup.yaml")),
#                     help="Fleet config (optional)")
#     ap.add_argument("--dt", type=float, default=0.05,
#                     help="Upload sample period (s); 0.05 = ESP nav loop rate")
#     ap.add_argument("--max-speed", type=float, default=0.30,
#                     help="Refuse faster trajectories (m/s); keep below MAV_CMD_SPEED_CAP_MS")
#     ap.add_argument("--takeoff", action="store_true", help="Send CMD_START (arm + take off) first")
#     ap.add_argument("--takeoff-wait", type=float, default=8.0,
#                     help="Seconds from CMD_START to trajectory upload")
#     ap.add_argument("--finish", choices=["land", "hold"], default="land",
#                     help="Action after the trajectory")
#     ap.add_argument("--confirm", action="store_true",
#                     help="Require typing TRAJ-<drone id> before sending any flight commands")
#     ap.add_argument("--tag-port", type=int, default=5008)
#     ap.add_argument("--id-margin", type=float, default=25.0)
#     ap.add_argument("--frames", type=int, default=3,
#                     help="Clean frames (hamming<=1, margin>id-margin) before a tag registers")
#     ap.add_argument("--reset-gap", type=float, default=1.0,
#                     help="Frame counter resets if a tag is unseen (cleanly) this long (s)")
#     ap.add_argument("--camera-pitch-deg", type=float, default=45.0)
#     ap.add_argument("--camera-forward", type=float, default=0.02)
#     ap.add_argument("--return-start", action="store_true",
#                     help="After completion, use CMD_GOTO to return to sampled start XY at firmware cruise altitude")
#     ap.add_argument("--return-timeout", type=float, default=120.0)
#     ap.add_argument("--tag-log", default="registered_tags.csv",
#                     help="CSV of registered IDs and estimated local-NED coordinates")
#     ap.add_argument("--telem-port", type=int, default=5005)
#     ap.add_argument("--cmd-port", type=int, default=5006)
#     args = ap.parse_args()
#     if not all(np.isfinite(v) for v in (args.id_margin,args.camera_pitch_deg,args.camera_forward)) or args.id_margin < 0:
#         raise SystemExit("Camera settings and margin must be finite; margin non-negative")
#     if not np.isfinite(args.return_timeout) or args.return_timeout <= 0:
#         raise SystemExit("--return-timeout must be positive and finite")

#     dt_ms = round(args.dt * 1000)
#     if not 1 <= dt_ms <= 1000:
#         raise SystemExit("--dt must be 0.001-1.0 s")
#     pts = load_trajectory(args.trajectory, dt_ms / 1000)
#     duration = (len(pts) - 1) * dt_ms / 1000
#     vmax = np.linalg.norm(np.diff(pts, axis=0), axis=1).max() * 1000 / dt_ms
#     log.info("%s: %d points, %.1f s, max speed %.3f m/s",
#              args.trajectory, len(pts), duration, vmax)
#     if vmax > args.max_speed + 1e-6:
#         raise SystemExit(f"max speed {vmax:.3f} m/s exceeds --max-speed {args.max_speed} m/s")
#     if len(pts) > TRAJ_MAX_PTS:
#         raise SystemExit(f"{len(pts)} points > {TRAJ_MAX_PTS}; use a larger --dt or a shorter trajectory")

#     latest = {}
#     registry = {}
#     registry_lock = threading.Lock()

#     def save_tags():
#         with registry_lock:
#             rows = [dict(drone_id=args.drone_id, **registry[k]) for k in sorted(registry)]
#         path = Path(args.tag_log)
#         path.parent.mkdir(parents=True, exist_ok=True)
#         temp = path.with_suffix(path.suffix + ".tmp")
#         with temp.open("w", newline="") as stream:
#             writer = csv.DictWriter(stream, fieldnames=["drone_id", "tag_id", "localized",
#                                                        "observations", "x", "y", "z"])
#             writer.writeheader()
#             writer.writerows(rows)
#         temp.replace(path)

#     # Create a header-only file immediately, even if no tags are registered.
#     save_tags()
#     log.info("Tag CSV: %s", Path(args.tag_log).resolve())

#     def on_survey(did, entries):
#         if did != args.drone_id:
#             return
#         latest["survey_at"] = time.monotonic()
#         changed = False
#         with registry_lock:
#             for entry in entries:
#                 old = registry.get(entry["tag_id"])
#                 registry[entry["tag_id"]] = entry
#                 if old is None or old["localized"] != entry["localized"]:
#                     changed = True
#                     if entry["localized"]:
#                         log.info("TAG %d REGISTERED ON LAPTOP: estimated telemetry-frame x=%.2f y=%.2f m (level-flight approximation)",
#                                  entry["tag_id"], entry["x"], entry["y"])
#                     else:
#                         log.info("TAG %d REGISTERED: coordinates pending (pose not yet valid)", entry["tag_id"])
#         if changed:
#             save_tags()

#     def on_telemetry(pkt, _ip):
#         if pkt.drone_id == args.drone_id:
#             latest["pkt"] = pkt
#             latest["received_at"] = time.monotonic()

#     def wait_for(cond, timeout_s: float) -> bool:
#         deadline = time.monotonic() + timeout_s
#         while time.monotonic() < deadline:
#             if ("pkt" in latest and time.monotonic() - latest["received_at"] < 3.0
#                     and cond(latest["pkt"])):
#                 return True
#             time.sleep(0.05)
#         return False

#     def send(cmd_type: int, name: str) -> None:
#         if not comms.send_command(args.drone_id, CommandPacket(cmd_type)):
#             raise RuntimeError(f"{name} not sent: drone {args.drone_id} IP unknown")
#         log.info("%s sent", name)

#     comms = CommsNode(
#         listen_port=args.telem_port,
#         cmd_port=args.cmd_port,
#         config_path=args.config if Path(args.config).exists() else None,
#     )
#     comms.on_telemetry(on_telemetry)
#     tags = HostTagRegistry(args.drone_id,
#         lambda: (latest.get("pkt"), latest.get("received_at", 0)), on_survey,
#         port=args.tag_port, min_margin=args.id_margin, frames=args.frames,
#         camera_pitch_deg=args.camera_pitch_deg, camera_forward=args.camera_forward,
#         reset_gap=args.reset_gap)
#     tags.start()
#     comms.start()
#     flying = not args.takeoff   # without --takeoff the drone must already hover
#     try:
#         if not wait_for(lambda p: True, 30.0):
#             raise RuntimeError(f"no telemetry from drone {args.drone_id}")

#         if args.confirm:
#             answer = input(f"Type TRAJ-{args.drone_id} to fly {args.trajectory}: ").strip()
#             if answer != f"TRAJ-{args.drone_id}":
#                 log.info("confirmation did not match; trajectory cancelled")
#                 return

#         if not wait_for(lambda p: time.monotonic()-tags.frame_at < 3.0, 5.0):
#             raise RuntimeError("No live AprilTag debug frames on UDP 5008. Check drone IP/network and existing camera debug support; no new survey packets are used.")

#         if args.takeoff:
#             send(CMD_START, "CMD_START")
#             flying = True
#             time.sleep(args.takeoff_wait)

#         # Upload, then start; retry if the drone did not switch to TRAJ.
#         traj_id = random.randint(1, 255)
#         packets = build_traj_packets(traj_id, pts)
#         start = build_traj_start(traj_id, dt_ms)
#         for attempt in range(1, 6):
#             for p in packets:
#                 comms.send_raw(args.drone_id, p)
#                 time.sleep(UPLOAD_GAP_S)
#             if time.monotonic()-latest["received_at"] > 3:
#                 raise RuntimeError("Telemetry stale before trajectory start")
#             origin = (latest["pkt"].ned_x, latest["pkt"].ned_y)
#             comms.send_raw(args.drone_id, start)
#             if wait_for(lambda p: p.nav_state == NAV_TRAJ, 1.5):
#                 break
#             log.warning("attempt %d: trajectory not started (see ESP log)", attempt)
#         else:
#             raise RuntimeError("drone refused the trajectory")

#         log.info("playing trajectory %d (%.1f s)", traj_id, duration)
#         # Firmware may slow playback to 25% while reading rejected tag candidates.
#         end = time.monotonic() + duration / 0.25 + 15.0
#         while True:
#             if tags.error:
#                 raise RuntimeError("Tag monitor failed: " + tags.error)
#             if time.monotonic()-tags.frame_at > 3.0:
#                 raise RuntimeError("Camera debug frames stale during survey")
#             if time.monotonic() - latest["received_at"] > 3.0:
#                 raise RuntimeError("Telemetry stale during trajectory playback")
#             p = latest["pkt"]
#             if p.nav_state == NAV_ARRIVED:
#                 break
#             if p.nav_state != NAV_TRAJ:
#                 raise RuntimeError(f"Trajectory interrupted: {p.nav_state_name}")
#             if time.monotonic() >= end:
#                 raise RuntimeError("Trajectory playback exceeded slow-flight timeout")
#             log.info("pos (%.2f, %.2f)", p.ned_x, p.ned_y)
#             time.sleep(1.0)
#         log.info("trajectory ended, nav state %s", latest["pkt"].nav_state_name)

#         if args.return_start:
#             def return_command():
#                 if not comms.send_command(args.drone_id, CommandPacket(
#                         CMD_GOTO, goal_x=origin[0], goal_y=origin[1])):
#                     raise RuntimeError("Could not send return waypoint")
#             log.info("Returning to sampled start XY (%.2f, %.2f)", *origin)
#             return_command()
#             sent_at = last_sent = time.monotonic()
#             deadline = sent_at + args.return_timeout
#             inside_since = None
#             while True:
#                 now = time.monotonic()
#                 if now-latest["received_at"] > 3:
#                     raise RuntimeError("Telemetry stale during return")
#                 p = latest["pkt"]
#                 distance = math.hypot(p.ned_x-origin[0],p.ned_y-origin[1])
#                 if latest["received_at"] > sent_at and distance <= .25:
#                     if inside_since is None:
#                         inside_since = now
#                     if now-inside_since >= 1.0:
#                         log.info("Returned within 0.25 m of start XY for 1 second")
#                         break
#                 else:
#                     inside_since = None
#                 if now>=deadline:
#                     raise RuntimeError("Return timed out; requesting landing in place")
#                 if p.nav_state not in (NAV_ARRIVED,NAV_IDLE,NAV_FLYING,NAV_ROTATING):
#                     raise RuntimeError(f"Return interrupted: {p.nav_state_name}")
#                 if now-last_sent >= 2 and p.nav_state in (NAV_ARRIVED,NAV_IDLE):
#                     return_command()
#                     last_sent=now
#                 time.sleep(.1)

#         if args.finish == "land":
#             for _ in range(3):   # UDP: repeat LAND
#                 send(CMD_LAND, "CMD_LAND")
#         else:
#             send(CMD_HOLD, "CMD_HOLD")
#     except (Exception, KeyboardInterrupt):
#         if flying:
#             log.error("aborted — sending CMD_LAND")
#             for _ in range(3):
#                 comms.send_command(args.drone_id, CommandPacket(CMD_LAND))
#         raise
#     finally:
#         tags.stop()
#         comms.stop()
#         save_tags()
#         log.info("Saved %d registered tags to %s (telemetry-frame XY; z unavailable)",
#                  len(registry), Path(args.tag_log).resolve())


# if __name__ == "__main__":
#     main()



# #!/usr/bin/env python3
# """
# Fly a CSV trajectory smoothly on one drone.

# Laptop-only tag registration uses the existing UDP 5008 debug stream.
# No new survey firmware or return command is required.
# The whole trajectory is uploaded first; the ESP32 then plays it on its own
# 20 Hz clock (position + velocity feedforward to PX4), so WiFi latency and
# jitter cannot disturb the flight.

# CSV: columns t,x,y,z (s, NED m); a header row and '#' comments are allowed.
# The trajectory is flown relative to where the drone hovers when it starts.

# Example:
#     python3 laptop/send_trajectory.py --drone-id 2 --takeoff --confirm \
#         --trajectory trajectory/circle_traj.csv
# """

# import argparse
# import csv
# import logging
# import random
# import time
# import threading
# from pathlib import Path

# import numpy as np

# from comms import CommsNode
# from host_tag_registry import HostTagRegistry
# import math
# from protocol import (
#     CMD_HOLD,
#     CMD_LAND,
#     CMD_START,
#     CMD_GOTO,
#     NAV_TRAJ,
#     NAV_ARRIVED,
#     NAV_IDLE,
#     NAV_FLYING,
#     NAV_ROTATING,
#     TRAJ_MAX_PTS,
#     CommandPacket,
#     build_traj_packets,
#     build_traj_start,
# )

# logging.basicConfig(
#     level=logging.INFO,
#     format="%(asctime)s | %(levelname)s | %(message)s",
#     datefmt="%H:%M:%S",
# )
# log = logging.getLogger("send_trajectory_no_flash")

# UPLOAD_GAP_S = 0.025   # ESP drains commands at 10 Hz from a 6-packet UDP queue


# def load_trajectory(path: str, dt: float) -> np.ndarray:
#     """Read t,x,y,z, resample every dt, return offsets from the first point."""
#     data = np.atleast_2d(np.genfromtxt(path, delimiter=",", comments="#"))
#     data = data[~np.isnan(data).any(axis=1)]           # drops the header row
#     if data.shape[0] < 2 or data.shape[1] < 4:
#         raise SystemExit(f"{path}: need at least 2 rows of t,x,y,z")
#     t = data[:, 0]
#     if np.any(np.diff(t) <= 0):
#         raise SystemExit(f"{path}: time column must be strictly increasing")
#     tq = np.arange(t[0], t[-1] + 1e-9, dt)
#     pts = np.column_stack([np.interp(tq, t, data[:, c]) for c in (1, 2, 3)])
#     return pts - pts[0]


# def main() -> None:
#     ap = argparse.ArgumentParser(description="Upload a CSV trajectory to one drone and fly it")
#     ap.add_argument("--drone-id", type=int, required=True)
#     ap.add_argument("--trajectory", required=True, help="CSV with columns t,x,y,z (s, NED m)")
#     ap.add_argument("--config", default=str(Path(__file__).with_name("setup.yaml")),
#                     help="Fleet config (optional)")
#     ap.add_argument("--dt", type=float, default=0.05,
#                     help="Upload sample period (s); 0.05 = ESP nav loop rate")
#     ap.add_argument("--max-speed", type=float, default=0.30,
#                     help="Refuse faster trajectories (m/s); keep below MAV_CMD_SPEED_CAP_MS")
#     ap.add_argument("--takeoff", action="store_true", help="Send CMD_START (arm + take off) first")
#     ap.add_argument("--takeoff-wait", type=float, default=8.0,
#                     help="Seconds from CMD_START to trajectory upload")
#     ap.add_argument("--finish", choices=["land", "hold"], default="land",
#                     help="Action after the trajectory")
#     ap.add_argument("--confirm", action="store_true",
#                     help="Require typing TRAJ-<drone id> before sending any flight commands")
#     ap.add_argument("--tag-port", type=int, default=5008)
#     ap.add_argument("--id-margin", type=float, default=25.0)
#     ap.add_argument("--frames", type=int, default=3,
#                     help="Clean frames (hamming<=1, margin>id-margin) before a tag registers")
#     ap.add_argument("--reset-gap", type=float, default=1.0,
#                     help="Frame counter resets if a tag is unseen (cleanly) this long (s)")
#     ap.add_argument("--camera-pitch-deg", type=float, default=45.0)
#     ap.add_argument("--camera-forward", type=float, default=0.02)
#     ap.add_argument("--return-start", action="store_true",
#                     help="After completion, use CMD_GOTO to return to sampled start XY at firmware cruise altitude")
#     ap.add_argument("--return-timeout", type=float, default=120.0)
#     ap.add_argument("--tag-log", default="registered_tags.csv",
#                     help="CSV of registered IDs and estimated local-NED coordinates")
#     ap.add_argument("--telem-port", type=int, default=5005)
#     ap.add_argument("--cmd-port", type=int, default=5006)
#     args = ap.parse_args()
#     if not all(np.isfinite(v) for v in (args.id_margin,args.camera_pitch_deg,args.camera_forward)) or args.id_margin < 0:
#         raise SystemExit("Camera settings and margin must be finite; margin non-negative")
#     if not np.isfinite(args.return_timeout) or args.return_timeout <= 0:
#         raise SystemExit("--return-timeout must be positive and finite")

#     dt_ms = round(args.dt * 1000)
#     if not 1 <= dt_ms <= 1000:
#         raise SystemExit("--dt must be 0.001-1.0 s")
#     pts = load_trajectory(args.trajectory, dt_ms / 1000)
#     duration = (len(pts) - 1) * dt_ms / 1000
#     vmax = np.linalg.norm(np.diff(pts, axis=0), axis=1).max() * 1000 / dt_ms
#     log.info("%s: %d points, %.1f s, max speed %.3f m/s",
#              args.trajectory, len(pts), duration, vmax)
#     if vmax > args.max_speed + 1e-6:
#         raise SystemExit(f"max speed {vmax:.3f} m/s exceeds --max-speed {args.max_speed} m/s")
#     if len(pts) > TRAJ_MAX_PTS:
#         raise SystemExit(f"{len(pts)} points > {TRAJ_MAX_PTS}; use a larger --dt or a shorter trajectory")

#     latest = {}
#     registry = {}
#     registry_lock = threading.Lock()

#     def save_tags():
#         with registry_lock:
#             rows = [dict(drone_id=args.drone_id, **registry[k]) for k in sorted(registry)]
#         path = Path(args.tag_log)
#         path.parent.mkdir(parents=True, exist_ok=True)
#         temp = path.with_suffix(path.suffix + ".tmp")
#         with temp.open("w", newline="") as stream:
#             writer = csv.DictWriter(stream, fieldnames=["drone_id", "tag_id", "localized",
#                                                        "observations", "x", "y", "z"])
#             writer.writeheader()
#             writer.writerows(rows)
#         temp.replace(path)

#     # Create a header-only file immediately, even if no tags are registered.
#     save_tags()
#     log.info("Tag CSV: %s", Path(args.tag_log).resolve())

#     def on_survey(did, entries):
#         if did != args.drone_id:
#             return
#         latest["survey_at"] = time.monotonic()
#         changed = False
#         with registry_lock:
#             for entry in entries:
#                 old = registry.get(entry["tag_id"])
#                 registry[entry["tag_id"]] = entry
#                 if old is None or old["localized"] != entry["localized"]:
#                     changed = True
#                     if entry["localized"]:
#                         log.info("TAG %d REGISTERED ON LAPTOP: estimated telemetry-frame x=%.2f y=%.2f m (level-flight approximation)",
#                                  entry["tag_id"], entry["x"], entry["y"])
#                     else:
#                         log.info("TAG %d REGISTERED: coordinates pending (pose not yet valid)", entry["tag_id"])
#         if changed:
#             save_tags()

#     def on_telemetry(pkt, _ip):
#         if pkt.drone_id == args.drone_id:
#             latest["pkt"] = pkt
#             latest["received_at"] = time.monotonic()

#     def wait_for(cond, timeout_s: float) -> bool:
#         deadline = time.monotonic() + timeout_s
#         while time.monotonic() < deadline:
#             if ("pkt" in latest and time.monotonic() - latest["received_at"] < 3.0
#                     and cond(latest["pkt"])):
#                 return True
#             time.sleep(0.05)
#         return False

#     def send(cmd_type: int, name: str) -> None:
#         if not comms.send_command(args.drone_id, CommandPacket(cmd_type)):
#             raise RuntimeError(f"{name} not sent: drone {args.drone_id} IP unknown")
#         log.info("%s sent", name)

#     comms = CommsNode(
#         listen_port=args.telem_port,
#         cmd_port=args.cmd_port,
#         config_path=args.config if Path(args.config).exists() else None,
#     )
#     comms.on_telemetry(on_telemetry)
#     tags = HostTagRegistry(args.drone_id,
#         lambda: (latest.get("pkt"), latest.get("received_at", 0)), on_survey,
#         port=args.tag_port, min_margin=args.id_margin, frames=args.frames,
#         camera_pitch_deg=args.camera_pitch_deg, camera_forward=args.camera_forward,
#         reset_gap=args.reset_gap)
#     tags.start()
#     comms.start()
#     flying = not args.takeoff   # without --takeoff the drone must already hover
#     try:
#         if not wait_for(lambda p: True, 30.0):
#             raise RuntimeError(f"no telemetry from drone {args.drone_id}")

#         if args.confirm:
#             answer = input(f"Type TRAJ-{args.drone_id} to fly {args.trajectory}: ").strip()
#             if answer != f"TRAJ-{args.drone_id}":
#                 log.info("confirmation did not match; trajectory cancelled")
#                 return

#         if not wait_for(lambda p: time.monotonic()-tags.frame_at < 3.0, 5.0):
#             raise RuntimeError("No live AprilTag debug frames on UDP 5008. Check drone IP/network and existing camera debug support; no new survey packets are used.")

#         # Capture the launch point BEFORE takeoff so return-to-start lands where
#         # the drone was placed, not where it drifted to during takeoff/settle.
#         home = (latest["pkt"].ned_x, latest["pkt"].ned_y)
#         log.info("Home (pre-takeoff) captured at (%.2f, %.2f)", *home)

#         if args.takeoff:
#             send(CMD_START, "CMD_START")
#             flying = True
#             time.sleep(args.takeoff_wait)

#         # Upload, then start; retry if the drone did not switch to TRAJ.
#         traj_id = random.randint(1, 255)
#         packets = build_traj_packets(traj_id, pts)
#         start = build_traj_start(traj_id, dt_ms)
#         for attempt in range(1, 6):
#             for p in packets:
#                 comms.send_raw(args.drone_id, p)
#                 time.sleep(UPLOAD_GAP_S)
#             if time.monotonic()-latest["received_at"] > 3:
#                 raise RuntimeError("Telemetry stale before trajectory start")
#             origin = (latest["pkt"].ned_x, latest["pkt"].ned_y)
#             comms.send_raw(args.drone_id, start)
#             if wait_for(lambda p: p.nav_state == NAV_TRAJ, 1.5):
#                 break
#             log.warning("attempt %d: trajectory not started (see ESP log)", attempt)
#         else:
#             raise RuntimeError("drone refused the trajectory")

#         log.info("playing trajectory %d (%.1f s)", traj_id, duration)
#         # Firmware may slow playback to 25% while reading rejected tag candidates.
#         end = time.monotonic() + duration / 0.25 + 15.0
#         while True:
#             if tags.error:
#                 raise RuntimeError("Tag monitor failed: " + tags.error)
#             if time.monotonic()-tags.frame_at > 3.0:
#                 raise RuntimeError("Camera debug frames stale during survey")
#             if time.monotonic() - latest["received_at"] > 3.0:
#                 raise RuntimeError("Telemetry stale during trajectory playback")
#             p = latest["pkt"]
#             if p.nav_state == NAV_ARRIVED:
#                 break
#             if p.nav_state != NAV_TRAJ:
#                 raise RuntimeError(f"Trajectory interrupted: {p.nav_state_name}")
#             if time.monotonic() >= end:
#                 raise RuntimeError("Trajectory playback exceeded slow-flight timeout")
#             log.info("pos (%.2f, %.2f)", p.ned_x, p.ned_y)
#             time.sleep(1.0)
#         log.info("trajectory ended, nav state %s", latest["pkt"].nav_state_name)

#         if args.return_start:
#             def return_command():
#                 if not comms.send_command(args.drone_id, CommandPacket(
#                         CMD_GOTO, goal_x=home[0], goal_y=home[1])):
#                     raise RuntimeError("Could not send return waypoint")
#             log.info("Returning to launch point (%.2f, %.2f)", *home)
#             return_command()
#             sent_at = last_sent = time.monotonic()
#             deadline = sent_at + args.return_timeout
#             inside_since = None
#             while True:
#                 now = time.monotonic()
#                 if now-latest["received_at"] > 3:
#                     raise RuntimeError("Telemetry stale during return")
#                 p = latest["pkt"]
#                 distance = math.hypot(p.ned_x-home[0],p.ned_y-home[1])
#                 if latest["received_at"] > sent_at and distance <= .25:
#                     if inside_since is None:
#                         inside_since = now
#                     if now-inside_since >= 1.0:
#                         log.info("Returned within 0.25 m of start XY for 1 second")
#                         break
#                 else:
#                     inside_since = None
#                 if now>=deadline:
#                     raise RuntimeError("Return timed out; requesting landing in place")
#                 if p.nav_state not in (NAV_ARRIVED,NAV_IDLE,NAV_FLYING,NAV_ROTATING):
#                     raise RuntimeError(f"Return interrupted: {p.nav_state_name}")
#                 if now-last_sent >= 2 and p.nav_state in (NAV_ARRIVED,NAV_IDLE):
#                     return_command()
#                     last_sent=now
#                 time.sleep(.1)

#         if args.finish == "land":
#             for _ in range(3):   # UDP: repeat LAND
#                 send(CMD_LAND, "CMD_LAND")
#         else:
#             send(CMD_HOLD, "CMD_HOLD")
#     except (Exception, KeyboardInterrupt):
#         if flying:
#             log.error("aborted — sending CMD_LAND")
#             for _ in range(3):
#                 comms.send_command(args.drone_id, CommandPacket(CMD_LAND))
#         raise
#     finally:
#         tags.stop()
#         comms.stop()
#         save_tags()
#         log.info("Saved %d registered tags to %s (telemetry-frame XY; z unavailable)",
#                  len(registry), Path(args.tag_log).resolve())


# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3
"""
Fly a CSV trajectory smoothly on one drone.

Laptop-only tag registration uses the existing UDP 5008 debug stream.
No new survey firmware or return command is required.
The whole trajectory is uploaded first; the ESP32 then plays it on its own
20 Hz clock (position + velocity feedforward to PX4), so WiFi latency and
jitter cannot disturb the flight.

CSV: columns t,x,y,z (s, NED m); a header row and '#' comments are allowed.
The trajectory is flown relative to where the drone hovers when it starts.

Example:
    python3 laptop/send_trajectory.py --drone-id 2 --takeoff --confirm \
        --trajectory trajectory/circle_traj.csv
"""

import argparse
import csv
import logging
import random
import time
import threading
from pathlib import Path

import numpy as np

from comms import CommsNode
from host_tag_registry import HostTagRegistry
import math
from protocol import (
    CMD_HOLD,
    CMD_LAND,
    CMD_START,
    CMD_GOTO,
    NAV_TRAJ,
    NAV_ARRIVED,
    NAV_IDLE,
    NAV_FLYING,
    NAV_ROTATING,
    TRAJ_MAX_PTS,
    CommandPacket,
    build_traj_packets,
    build_traj_start,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("send_trajectory_no_flash")

UPLOAD_GAP_S = 0.025   # ESP drains commands at 10 Hz from a 6-packet UDP queue


def load_trajectory(path: str, dt: float) -> np.ndarray:
    """Read t,x,y,z, resample every dt, return offsets from the first point."""
    data = np.atleast_2d(np.genfromtxt(path, delimiter=",", comments="#"))
    data = data[~np.isnan(data).any(axis=1)]           # drops the header row
    if data.shape[0] < 2 or data.shape[1] < 4:
        raise SystemExit(f"{path}: need at least 2 rows of t,x,y,z")
    t = data[:, 0]
    if np.any(np.diff(t) <= 0):
        raise SystemExit(f"{path}: time column must be strictly increasing")
    tq = np.arange(t[0], t[-1] + 1e-9, dt)
    pts = np.column_stack([np.interp(tq, t, data[:, c]) for c in (1, 2, 3)])
    return pts - pts[0]


def main() -> None:
    ap = argparse.ArgumentParser(description="Upload a CSV trajectory to one drone and fly it")
    ap.add_argument("--drone-id", type=int, required=True)
    ap.add_argument("--trajectory", required=True, help="CSV with columns t,x,y,z (s, NED m)")
    ap.add_argument("--config", default=str(Path(__file__).with_name("setup.yaml")),
                    help="Fleet config (optional)")
    ap.add_argument("--dt", type=float, default=0.05,
                    help="Upload sample period (s); 0.05 = ESP nav loop rate")
    ap.add_argument("--max-speed", type=float, default=0.30,
                    help="Refuse faster trajectories (m/s); keep below MAV_CMD_SPEED_CAP_MS")
    ap.add_argument("--takeoff", action="store_true", help="Send CMD_START (arm + take off) first")
    ap.add_argument("--takeoff-wait", type=float, default=8.0,
                    help="Seconds from CMD_START to trajectory upload")
    ap.add_argument("--finish", choices=["land", "hold"], default="land",
                    help="Action after the trajectory")
    ap.add_argument("--confirm", action="store_true",
                    help="Require typing TRAJ-<drone id> before sending any flight commands")
    ap.add_argument("--tag-port", type=int, default=5008)
    ap.add_argument("--id-margin", type=float, default=25.0)
    ap.add_argument("--frames", type=int, default=3,
                    help="Clean frames (hamming<=1, margin>id-margin) before a tag registers")
    ap.add_argument("--reset-gap", type=float, default=1.0,
                    help="Frame counter resets if a tag is unseen (cleanly) this long (s)")
    ap.add_argument("--camera-pitch-deg", type=float, default=45.0)
    ap.add_argument("--camera-forward", type=float, default=0.02)
    ap.add_argument("--return-start", action="store_true",
                    help="After completion, use CMD_GOTO to return to sampled start XY at firmware cruise altitude")
    ap.add_argument("--return-offset-x", type=float, default=0.0,
                    help="Added to the return-home X target to compensate position-estimate drift (m). "
                         "If it lands short by +0.3 in x, pass 0.3.")
    ap.add_argument("--return-offset-y", type=float, default=0.0,
                    help="Added to the return-home Y target to compensate position-estimate drift (m).")
    ap.add_argument("--return-timeout", type=float, default=120.0)
    ap.add_argument("--tag-log", default="registered_tags.csv",
                    help="CSV of registered IDs and estimated local-NED coordinates")
    ap.add_argument("--telem-port", type=int, default=5005)
    ap.add_argument("--cmd-port", type=int, default=5006)
    args = ap.parse_args()
    if not all(np.isfinite(v) for v in (args.id_margin,args.camera_pitch_deg,args.camera_forward)) or args.id_margin < 0:
        raise SystemExit("Camera settings and margin must be finite; margin non-negative")
    if not np.isfinite(args.return_timeout) or args.return_timeout <= 0:
        raise SystemExit("--return-timeout must be positive and finite")

    dt_ms = round(args.dt * 1000)
    if not 1 <= dt_ms <= 1000:
        raise SystemExit("--dt must be 0.001-1.0 s")
    pts = load_trajectory(args.trajectory, dt_ms / 1000)
    duration = (len(pts) - 1) * dt_ms / 1000
    vmax = np.linalg.norm(np.diff(pts, axis=0), axis=1).max() * 1000 / dt_ms
    log.info("%s: %d points, %.1f s, max speed %.3f m/s",
             args.trajectory, len(pts), duration, vmax)
    if vmax > args.max_speed + 1e-6:
        raise SystemExit(f"max speed {vmax:.3f} m/s exceeds --max-speed {args.max_speed} m/s")
    if len(pts) > TRAJ_MAX_PTS:
        raise SystemExit(f"{len(pts)} points > {TRAJ_MAX_PTS}; use a larger --dt or a shorter trajectory")

    latest = {}
    registry = {}
    registry_lock = threading.Lock()

    def save_tags():
        with registry_lock:
            rows = [dict(drone_id=args.drone_id, **registry[k]) for k in sorted(registry)]
        path = Path(args.tag_log)
        path.parent.mkdir(parents=True, exist_ok=True)
        temp = path.with_suffix(path.suffix + ".tmp")
        with temp.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=["drone_id", "tag_id", "localized",
                                                       "observations", "x", "y", "z"])
            writer.writeheader()
            writer.writerows(rows)
        temp.replace(path)

    # Create a header-only file immediately, even if no tags are registered.
    save_tags()
    log.info("Tag CSV: %s", Path(args.tag_log).resolve())

    def on_survey(did, entries):
        if did != args.drone_id:
            return
        latest["survey_at"] = time.monotonic()
        changed = False
        with registry_lock:
            for entry in entries:
                old = registry.get(entry["tag_id"])
                registry[entry["tag_id"]] = entry
                if old is None or old["localized"] != entry["localized"]:
                    changed = True
                    if entry["localized"]:
                        log.info("TAG %d REGISTERED ON LAPTOP: estimated telemetry-frame x=%.2f y=%.2f m (level-flight approximation)",
                                 entry["tag_id"], entry["x"], entry["y"])
                    else:
                        log.info("TAG %d REGISTERED: coordinates pending (pose not yet valid)", entry["tag_id"])
        if changed:
            save_tags()

    def on_telemetry(pkt, _ip):
        if pkt.drone_id == args.drone_id:
            latest["pkt"] = pkt
            latest["received_at"] = time.monotonic()

    def wait_for(cond, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if ("pkt" in latest and time.monotonic() - latest["received_at"] < 3.0
                    and cond(latest["pkt"])):
                return True
            time.sleep(0.05)
        return False

    def send(cmd_type: int, name: str) -> None:
        if not comms.send_command(args.drone_id, CommandPacket(cmd_type)):
            raise RuntimeError(f"{name} not sent: drone {args.drone_id} IP unknown")
        log.info("%s sent", name)

    comms = CommsNode(
        listen_port=args.telem_port,
        cmd_port=args.cmd_port,
        config_path=args.config if Path(args.config).exists() else None,
    )
    comms.on_telemetry(on_telemetry)
    tags = HostTagRegistry(args.drone_id,
        lambda: (latest.get("pkt"), latest.get("received_at", 0)), on_survey,
        port=args.tag_port, min_margin=args.id_margin, frames=args.frames,
        camera_pitch_deg=args.camera_pitch_deg, camera_forward=args.camera_forward,
        reset_gap=args.reset_gap)
    tags.start()
    comms.start()
    flying = not args.takeoff   # without --takeoff the drone must already hover
    try:
        if not wait_for(lambda p: True, 30.0):
            raise RuntimeError(f"no telemetry from drone {args.drone_id}")

        if args.confirm:
            answer = input(f"Type TRAJ-{args.drone_id} to fly {args.trajectory}: ").strip()
            if answer != f"TRAJ-{args.drone_id}":
                log.info("confirmation did not match; trajectory cancelled")
                return

        if not wait_for(lambda p: time.monotonic()-tags.frame_at < 3.0, 5.0):
            raise RuntimeError("No live AprilTag debug frames on UDP 5008. Check drone IP/network and existing camera debug support; no new survey packets are used.")

        # Capture the launch point BEFORE takeoff so return-to-start lands where
        # the drone was placed, not where it drifted to during takeoff/settle.
        home = (latest["pkt"].ned_x, latest["pkt"].ned_y)
        log.info("Home (pre-takeoff) captured at (%.2f, %.2f)", *home)

        if args.takeoff:
            send(CMD_START, "CMD_START")
            flying = True
            time.sleep(args.takeoff_wait)

        # Upload, then start; retry if the drone did not switch to TRAJ.
        traj_id = random.randint(1, 255)
        packets = build_traj_packets(traj_id, pts)
        start = build_traj_start(traj_id, dt_ms)
        for attempt in range(1, 6):
            for p in packets:
                comms.send_raw(args.drone_id, p)
                time.sleep(UPLOAD_GAP_S)
            if time.monotonic()-latest["received_at"] > 3:
                raise RuntimeError("Telemetry stale before trajectory start")
            origin = (latest["pkt"].ned_x, latest["pkt"].ned_y)
            comms.send_raw(args.drone_id, start)
            if wait_for(lambda p: p.nav_state == NAV_TRAJ, 1.5):
                break
            log.warning("attempt %d: trajectory not started (see ESP log)", attempt)
        else:
            raise RuntimeError("drone refused the trajectory")

        log.info("playing trajectory %d (%.1f s)", traj_id, duration)
        # Firmware may slow playback to 25% while reading rejected tag candidates.
        end = time.monotonic() + duration / 0.25 + 15.0
        while True:
            if tags.error:
                raise RuntimeError("Tag monitor failed: " + tags.error)
            if time.monotonic()-tags.frame_at > 3.0:
                raise RuntimeError("Camera debug frames stale during survey")
            if time.monotonic() - latest["received_at"] > 3.0:
                raise RuntimeError("Telemetry stale during trajectory playback")
            p = latest["pkt"]
            if p.nav_state == NAV_ARRIVED:
                break
            if p.nav_state != NAV_TRAJ:
                raise RuntimeError(f"Trajectory interrupted: {p.nav_state_name}")
            if time.monotonic() >= end:
                raise RuntimeError("Trajectory playback exceeded slow-flight timeout")
            log.info("pos (%.2f, %.2f)", p.ned_x, p.ned_y)
            time.sleep(1.0)
        log.info("trajectory ended, nav state %s", latest["pkt"].nav_state_name)

        if args.return_start:
            target = (home[0] + args.return_offset_x, home[1] + args.return_offset_y)
            def return_command():
                if not comms.send_command(args.drone_id, CommandPacket(
                        CMD_GOTO, goal_x=target[0], goal_y=target[1])):
                    raise RuntimeError("Could not send return waypoint")
            if args.return_offset_x or args.return_offset_y:
                log.info("Returning to launch point (%.2f, %.2f) + drift offset "
                         "(%.2f, %.2f) = target (%.2f, %.2f)",
                         home[0], home[1], args.return_offset_x, args.return_offset_y, *target)
            else:
                log.info("Returning to launch point (%.2f, %.2f)", *target)
            return_command()
            sent_at = last_sent = time.monotonic()
            deadline = sent_at + args.return_timeout
            inside_since = None
            while True:
                now = time.monotonic()
                if now-latest["received_at"] > 3:
                    raise RuntimeError("Telemetry stale during return")
                p = latest["pkt"]
                distance = math.hypot(p.ned_x-target[0],p.ned_y-target[1])
                if latest["received_at"] > sent_at and distance <= .25:
                    if inside_since is None:
                        inside_since = now
                    if now-inside_since >= 1.0:
                        log.info("Returned within 0.25 m of start XY for 1 second")
                        break
                else:
                    inside_since = None
                if now>=deadline:
                    raise RuntimeError("Return timed out; requesting landing in place")
                if p.nav_state not in (NAV_ARRIVED,NAV_IDLE,NAV_FLYING,NAV_ROTATING):
                    raise RuntimeError(f"Return interrupted: {p.nav_state_name}")
                if now-last_sent >= 2 and p.nav_state in (NAV_ARRIVED,NAV_IDLE):
                    return_command()
                    last_sent=now
                time.sleep(.1)

        if args.finish == "land":
            for _ in range(3):   # UDP: repeat LAND
                send(CMD_LAND, "CMD_LAND")
        else:
            send(CMD_HOLD, "CMD_HOLD")
    except (Exception, KeyboardInterrupt):
        if flying:
            log.error("aborted — sending CMD_LAND")
            for _ in range(3):
                comms.send_command(args.drone_id, CommandPacket(CMD_LAND))
        raise
    finally:
        tags.stop()
        comms.stop()
        save_tags()
        log.info("Saved %d registered tags to %s (telemetry-frame XY; z unavailable)",
                 len(registry), Path(args.tag_log).resolve())


if __name__ == "__main__":
    main()
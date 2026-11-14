#!/usr/bin/env python3
"""
Reactive lawnmower mission — laptop-streamed waypoints with tag-triggered hover.

Why this exists (and why it is NOT send_trajectory.py):
    send_trajectory.py uploads the whole CSV and the ESP32 plays it on its own
    20 Hz clock (traj_tick in nav_task.c). Once CMD_TRAJ_START is sent the laptop
    has NO control over playback speed — it cannot pause or slow for a tag. The
    "firmware slows to 25%" comment in send_trajectory.py is stale: the firmware
    does not do that.

    To get dynamic speed / hover-on-tag WITHOUT any firmware change, the laptop
    must own the clock. This script streams the lawnmower as a sequence of
    CMD_GOTO waypoints (the firmware's existing VFH nav flies each leg), and
    between/along legs it watches the live AprilTag stream. The instant an
    unresolved target tag enters view, it issues CMD_HOLD (hover), waits until
    the firmware localises that tag (or a per-tag timeout), then resumes the leg.

    All firmware code is untouched. Only mission logic changes.

Lane spacing:
    Lanes are clustered toward the y-borders (cosine / Chebyshev distribution)
    and spread out in the middle, because tags are usually placed near borders.
    --edge-bias 0 = uniform spacing, 1 = maximum edge clustering.

Usage:
    python run_reactive_lawnmower.py --drone-id 5 --takeoff --confirm \
        --lanes 7 --edge-bias 0.7 --margin 0.4 --expected-tags 4
"""

import argparse
import csv
import logging
import math
import socket
import sys
import threading
import time
from pathlib import Path

import yaml

from comms import CommsNode
from protocol import (
    CMD_GOTO, CMD_HOLD, CMD_LAND, CMD_START, CommandPacket,
    NAV_ARRIVED, NAV_IDLE, NAV_FLYING, NAV_ROTATING,
    AT_DEBUG_PORT, parse_at_debug,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("reactive")

# Firmware gate constants (keep in sync with main/at_detect.h)
AT_ID_MIN_MARGIN = 25.0
AT_ID_MAX_HAMMING = 1

# Mission tuning
ARRIVE_RADIUS_M   = 0.30   # advance to next waypoint when this close
HOVER_SETTLE_S    = 0.4    # min hover before we start checking localisation
HOVER_MAX_S       = 6.0    # give up hovering for a tag after this (partial credit)
RESUME_GRACE_S    = 1.0    # after resolving, ignore re-triggers this long
RETURN_RADIUS_M   = 0.25
RETURN_TIMEOUT_S  = 60.0
TELEM_STALE_S     = 3.0
PROGRESS_LOG_S    = 1.5    # log pos/goal/distance this often while flying a leg
WAYPOINT_TIMEOUT_S = 40.0  # if a leg isn't reached in this long, warn + advance


# ---------------------------------------------------------------------------
# Variable-spacing lawnmower generator (edge-dense)
# ---------------------------------------------------------------------------
def generate_variable_lawnmower(min_x, max_x, min_y, max_y,
                                margin, lanes, edge_bias):
    """Return [(x, y), ...] sweep points in arena coords, lanes along x.

    edge_bias in [0,1]: 0 = uniform y-spacing, 1 = lanes clustered at the
    y-borders (cosine distribution), middle lanes spread wider apart.
    """
    if not (0.0 <= edge_bias <= 1.0):
        raise ValueError("edge-bias must be in [0, 1]")
    if lanes < 1:
        raise ValueError("lanes must be >= 1")
    x0, x1 = min_x + margin, max_x - margin
    y0, y1 = min_y + margin, max_y - margin
    if x0 >= x1 or y0 >= y1:
        raise ValueError("margin too large for arena")

    pts = []
    for i in range(lanes + 1):
        f = i / lanes
        cheb = 0.5 * (1.0 - math.cos(math.pi * f))   # dense near 0 and 1
        g = (1.0 - edge_bias) * f + edge_bias * cheb
        y = y0 + (y1 - y0) * g
        ends = (x0, x1) if i % 2 == 0 else (x1, x0)
        pts.extend((x, y) for x in ends)
    return pts


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drone-id", type=int, required=True)
    ap.add_argument("--config", default=str(Path(__file__).with_name("setup.yaml")))
    ap.add_argument("--lanes", type=int, default=6, help="Number of sweep lanes")
    ap.add_argument("--edge-bias", type=float, default=0.65,
                    help="0=uniform y-spacing, 1=cluster lanes at borders")
    ap.add_argument("--margin", type=float, default=0.4,
                    help="Distance from arena border to outer lanes (m)")
    ap.add_argument("--expected-tags", type=int, default=0,
                    help="If >0, finish early once this many tags are localised")
    ap.add_argument("--takeoff", action="store_true", help="Send CMD_START first")
    ap.add_argument("--takeoff-wait", type=float, default=8.0)
    ap.add_argument("--confirm", action="store_true",
                    help="Require typing GO-<drone id> before any flight command")
    ap.add_argument("--tag-log", default="registered_tags.csv")
    ap.add_argument("--telem-port", type=int, default=5005)
    ap.add_argument("--cmd-port", type=int, default=5006)
    ap.add_argument("--dry-run", action="store_true",
                    help="Print the waypoint plan and exit (no connection)")
    args = ap.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text())
    arena = cfg["arena"]
    nav_tag_ids = {int(t) for t in cfg.get("nav_tags", {})}  # landmarks: never hover
    log.info("Nav (landmark) tags excluded from hover: %s", sorted(nav_tag_ids) or "none")

    sweep = generate_variable_lawnmower(
        arena["min_x"], arena["max_x"], arena["min_y"], arena["max_y"],
        args.margin, args.lanes, args.edge_bias)
    # Waypoints are OFFSETS from the start-hover point (same convention as the
    # CSV path: prepend (0,0)). Absolute goal = sampled origin + offset.
    waypoints = [(0.0, 0.0)] + sweep

    log.info("Lawnmower: %d lanes, edge-bias %.2f, margin %.2fm → %d waypoints",
             args.lanes, args.edge_bias, args.margin, len(waypoints))
    lane_ys = sorted({round(y, 2) for _, y in sweep})
    gaps = [round(b - a, 2) for a, b in zip(lane_ys, lane_ys[1:])]
    log.info("Lane y-positions: %s", lane_ys)
    log.info("Lane gaps (border→middle→border): %s", gaps)

    if args.dry_run:
        for i, (x, y) in enumerate(waypoints):
            log.info("  wp %2d  offset (%.2f, %.2f)", i, x, y)
        return

    # -------- Shared state --------
    latest = {"pkt": None, "received_at": 0.0}
    registry = {}                 # tag_id -> survey entry dict
    registry_lock = threading.Lock()
    resolved = set()              # tag ids we are done reading
    in_view = {}                  # tag_id -> monotonic time last seen in 5008 stream
    in_view_lock = threading.Lock()
    events = []                   # (ts_str, text) mission event log

    def stamp():
        t = time.time()
        return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int((t % 1) * 1000):03d}"

    def event(text):
        s = stamp()
        events.append((s, text))
        log.info("EVENT [%s] %s", s, text)

    # -------- Survey callback (5005): localisation = resolved --------
    def on_survey(did, entries):
        if did != args.drone_id:
            return
        with registry_lock:
            for e in entries:
                tid = e["tag_id"]
                old = registry.get(tid)
                registry[tid] = e
                if e["localized"] and (old is None or not old["localized"]):
                    event(f"TAG {tid} LOCALISED  NED x={e['x']:.2f} y={e['y']:.2f} "
                          f"z={e['z']:.2f}  obs={e['observations']}")
                    resolved.add(tid)
                elif old is None:
                    event(f"TAG {tid} ID-confirmed (pose pending)  obs={e['observations']}")

    def on_telemetry(pkt, _ip):
        if pkt.drone_id == args.drone_id:
            latest["pkt"] = pkt
            latest["received_at"] = time.monotonic()

    # -------- 5008 live stream: early "tag in view" signal (optional) --------
    stop_evt = threading.Event()

    def live_listener(sock):
        while not stop_evt.is_set():
            try:
                data, _ = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            pkt = parse_at_debug(data)
            if pkt is None or pkt.drone_id != args.drone_id:
                continue
            now = time.monotonic()
            with in_view_lock:
                for d in pkt.detections:
                    if (d.hamming <= AT_ID_MAX_HAMMING and d.margin > AT_ID_MIN_MARGIN
                            and d.id not in nav_tag_ids):
                        in_view[d.id] = now

    live_sock = None
    try:
        live_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        live_sock.bind(("", AT_DEBUG_PORT))
        live_sock.settimeout(0.2)
        threading.Thread(target=live_listener, args=(live_sock,), daemon=True).start()
        log.info("Live tag stream bound on UDP %d (early hover trigger active)", AT_DEBUG_PORT)
    except OSError as exc:
        log.warning("UDP %d unavailable (%s) — hover triggers on survey only, "
                    "which is ~3 frames slower. Close tag_debug.py/tag_stream.py "
                    "to enable the fast trigger.", AT_DEBUG_PORT, exc)

    def unresolved_tag_in_view():
        """Return a target tag id currently in view and not yet resolved, else None."""
        now = time.monotonic()
        # Fast path: 5008 raw detections within the last 0.5 s.
        with in_view_lock:
            for tid, seen in in_view.items():
                if tid not in resolved and now - seen < 0.5:
                    return tid
        # Fallback: survey has it ID-confirmed but not localised.
        with registry_lock:
            for tid, e in registry.items():
                if tid not in resolved and not e["localized"] and tid not in nav_tag_ids:
                    return tid
        return None

    # -------- Comms --------
    comms = CommsNode(listen_port=args.telem_port, cmd_port=args.cmd_port,
                      config_path=args.config if Path(args.config).exists() else None)
    comms.on_telemetry(on_telemetry)
    if hasattr(comms, "on_survey"):
        comms.on_survey(on_survey)
    comms.start()

    def fresh():
        return (latest["pkt"] is not None
                and time.monotonic() - latest["received_at"] < TELEM_STALE_S)

    def wait_fresh(timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if fresh():
                return True
            time.sleep(0.05)
        return False

    def goto(gx, gy):
        comms.send_command(args.drone_id, CommandPacket(CMD_GOTO, goal_x=gx, goal_y=gy))

    def save_tags():
        with registry_lock:
            rows = [dict(drone_id=args.drone_id, **registry[k]) for k in sorted(registry)]
        p = Path(args.tag_log)
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["drone_id", "tag_id", "localized",
                                              "observations", "x", "y", "z"])
            w.writeheader()
            w.writerows(rows)

    flying = not args.takeoff
    try:
        if not wait_fresh(30.0):
            raise RuntimeError(f"No telemetry from drone {args.drone_id}")

        if args.confirm:
            if input(f"Type GO-{args.drone_id} to fly: ").strip() != f"GO-{args.drone_id}":
                log.info("cancelled")
                return

        if args.takeoff:
            event(f"CMD_START (autonomous takeoff) → drone {args.drone_id}")
            comms.send_command(args.drone_id, CommandPacket(CMD_START))
            flying = True
            time.sleep(args.takeoff_wait)

        if not wait_fresh(3.0):
            raise RuntimeError("No fresh position before starting sweep")
        origin = (latest["pkt"].ned_x, latest["pkt"].ned_y)
        event(f"sweep start — origin ({origin[0]:.2f}, {origin[1]:.2f})")

        # -------- Walk the lawnmower, hovering on tags --------
        for idx, (ox, oy) in enumerate(waypoints):
            gx, gy = origin[0] + ox, origin[1] + oy
            log.info("→ waypoint %d/%d  goal (%.2f, %.2f)", idx, len(waypoints) - 1, gx, gy)
            goto(gx, gy)
            leg_start = time.monotonic()
            last_prog = 0.0
            prev_dist = None

            while True:
                if not fresh():
                    raise RuntimeError("Telemetry stale mid-sweep")
                p = latest["pkt"]
                now = time.monotonic()
                dist = math.hypot(p.ned_x - gx, p.ned_y - gy)

                # Progress log — shows whether the drone is approaching or receding
                if now - last_prog >= PROGRESS_LOG_S:
                    trend = ""
                    if prev_dist is not None:
                        trend = "approaching" if dist < prev_dist - 0.02 else \
                                ("RECEDING" if dist > prev_dist + 0.02 else "holding")
                    log.info("   flying wp%d: pos (%.2f, %.2f) → goal (%.2f, %.2f)  "
                             "dist=%.2fm  nav=%s  %s",
                             idx, p.ned_x, p.ned_y, gx, gy, dist, p.nav_state_name, trend)
                    last_prog = now
                    prev_dist = dist

                # Per-leg timeout — never hang silently
                if now - leg_start > WAYPOINT_TIMEOUT_S:
                    log.warning("   wp%d not reached in %.0fs (dist=%.2fm, nav=%s) — "
                                "advancing to keep coverage going",
                                idx, WAYPOINT_TIMEOUT_S, dist, p.nav_state_name)
                    break

                # Early finish if we already have all expected tags
                if args.expected_tags > 0:
                    with registry_lock:
                        n_loc = sum(1 for e in registry.values() if e["localized"])
                    if n_loc >= args.expected_tags:
                        event(f"all {args.expected_tags} expected tags localised — ending sweep")
                        raise StopIteration

                # Tag-triggered hover
                tid = unresolved_tag_in_view()
                if tid is not None:
                    event(f"TAG {tid} in view — HOVER to read")
                    comms.send_command(args.drone_id, CommandPacket(CMD_HOLD))
                    hover_start = time.monotonic()
                    time.sleep(HOVER_SETTLE_S)
                    while time.monotonic() - hover_start < HOVER_MAX_S:
                        if tid in resolved:
                            break
                        time.sleep(0.1)
                    if tid not in resolved:
                        resolved.add(tid)   # timed out → partial credit, stop re-hovering
                        event(f"TAG {tid} hover timeout ({HOVER_MAX_S:.0f}s) — "
                              f"ID logged, pose may be missing")
                    else:
                        event(f"TAG {tid} read — resuming sweep")
                    goto(gx, gy)            # resume the current leg
                    time.sleep(RESUME_GRACE_S)
                    leg_start = time.monotonic()   # don't count hover against leg timeout
                    prev_dist = None
                    continue

                # Arrived at this waypoint?
                if math.hypot(p.ned_x - gx, p.ned_y - gy) < ARRIVE_RADIUS_M \
                        or p.nav_state == NAV_ARRIVED:
                    break
                time.sleep(0.2)

        event("sweep complete")

    except StopIteration:
        event("sweep ended early")
    except (Exception, KeyboardInterrupt) as exc:
        log.error("aborted: %s", exc)
        if flying:
            for _ in range(3):
                comms.send_command(args.drone_id, CommandPacket(CMD_LAND))
        comms.stop()
        if registry:
            save_tags()
        _report(events, registry, registry_lock, args)
        raise

    # -------- Return to start, land --------
    try:
        if flying:
            event(f"returning to start ({origin[0]:.2f}, {origin[1]:.2f})")
            goto(origin[0], origin[1])
            t_ret = time.monotonic()
            while time.monotonic() - t_ret < RETURN_TIMEOUT_S:
                if fresh():
                    p = latest["pkt"]
                    if math.hypot(p.ned_x - origin[0], p.ned_y - origin[1]) < RETURN_RADIUS_M \
                            or p.nav_state == NAV_ARRIVED:
                        event("returned to start")
                        break
                if (time.monotonic() - t_ret) % 4.0 < 0.2:
                    goto(origin[0], origin[1])
                time.sleep(0.2)
            else:
                event("return timeout — landing in place")

            event(f"CMD_LAND → drone {args.drone_id}")
            for _ in range(3):
                comms.send_command(args.drone_id, CommandPacket(CMD_LAND))
            time.sleep(2.0)
    finally:
        stop_evt.set()
        comms.stop()
        if live_sock is not None:
            live_sock.close()
        if registry:
            save_tags()
            log.info("Tag coordinates saved to %s", args.tag_log)
        _report(events, registry, registry_lock, args)


def _report(events, registry, registry_lock, args):
    log.info("")
    log.info("=" * 60)
    log.info("MISSION REPORT — drone %d", args.drone_id)
    log.info("=" * 60)
    with registry_lock:
        tags = dict(registry)
    log.info("Tags detected: %d | localised: %d",
             len(tags), sum(1 for e in tags.values() if e["localized"]))
    for tid in sorted(tags):
        e = tags[tid]
        if e["localized"]:
            log.info("  tag %2d  NED (%.2f, %.2f, %.2f)  obs=%d",
                     tid, e["x"], e["y"], e["z"], e["observations"])
        else:
            log.info("  tag %2d  (id confirmed, no pose)  obs=%d", tid, e["observations"])
    log.info("-" * 60)
    log.info("Event timeline:")
    for s, text in events:
        log.info("  [%s] %s", s, text)
    log.info("=" * 60)


if __name__ == "__main__":
    main()








#old code
# #!/usr/bin/env python3
# """
# Reactive lawnmower mission — laptop-streamed waypoints with tag-triggered hover.

# Why this exists (and why it is NOT send_trajectory.py):
#     send_trajectory.py uploads the whole CSV and the ESP32 plays it on its own
#     20 Hz clock (traj_tick in nav_task.c). Once CMD_TRAJ_START is sent the laptop
#     has NO control over playback speed — it cannot pause or slow for a tag. The
#     "firmware slows to 25%" comment in send_trajectory.py is stale: the firmware
#     does not do that.

#     To get dynamic speed / hover-on-tag WITHOUT any firmware change, the laptop
#     must own the clock. This script streams the lawnmower as a sequence of
#     CMD_GOTO waypoints (the firmware's existing VFH nav flies each leg), and
#     between/along legs it watches the live AprilTag stream. The instant an
#     unresolved target tag enters view, it issues CMD_HOLD (hover), waits until
#     the firmware localises that tag (or a per-tag timeout), then resumes the leg.

#     All firmware code is untouched. Only mission logic changes.

# Lane spacing:
#     Lanes are clustered toward the y-borders (cosine / Chebyshev distribution)
#     and spread out in the middle, because tags are usually placed near borders.
#     --edge-bias 0 = uniform spacing, 1 = maximum edge clustering.

# Usage:
#     python run_reactive_lawnmower.py --drone-id 5 --takeoff --confirm \
#         --lanes 7 --edge-bias 0.7 --margin 0.4 --expected-tags 4
# """

# import argparse
# import csv
# import logging
# import math
# import socket
# import sys
# import threading
# import time
# from pathlib import Path

# import yaml

# from comms import CommsNode
# from protocol import (
#     CMD_GOTO, CMD_HOLD, CMD_LAND, CMD_START, CommandPacket,
#     NAV_ARRIVED, NAV_IDLE, NAV_FLYING, NAV_ROTATING,
#     AT_DEBUG_PORT, parse_at_debug,
# )

# logging.basicConfig(
#     level=logging.INFO,
#     format="%(asctime)s.%(msecs)03d | %(message)s",
#     datefmt="%H:%M:%S",
# )
# log = logging.getLogger("reactive")

# # Firmware gate constants (keep in sync with main/at_detect.h)
# AT_ID_MIN_MARGIN = 25.0
# AT_ID_MAX_HAMMING = 1

# # Mission tuning
# ARRIVE_RADIUS_M   = 0.30   # advance to next waypoint when this close
# HOVER_SETTLE_S    = 0.4    # min hover before we start checking localisation
# HOVER_MAX_S       = 6.0    # give up hovering for a tag after this (partial credit)
# RESUME_GRACE_S    = 1.0    # after resolving, ignore re-triggers this long
# RETURN_RADIUS_M   = 0.25
# RETURN_TIMEOUT_S  = 60.0
# TELEM_STALE_S     = 3.0


# # ---------------------------------------------------------------------------
# # Variable-spacing lawnmower generator (edge-dense)
# # ---------------------------------------------------------------------------
# def generate_variable_lawnmower(min_x, max_x, min_y, max_y,
#                                 margin, lanes, edge_bias):
#     """Return [(x, y), ...] sweep points in arena coords, lanes along x.

#     edge_bias in [0,1]: 0 = uniform y-spacing, 1 = lanes clustered at the
#     y-borders (cosine distribution), middle lanes spread wider apart.
#     """
#     if not (0.0 <= edge_bias <= 1.0):
#         raise ValueError("edge-bias must be in [0, 1]")
#     if lanes < 1:
#         raise ValueError("lanes must be >= 1")
#     x0, x1 = min_x + margin, max_x - margin
#     y0, y1 = min_y + margin, max_y - margin
#     if x0 >= x1 or y0 >= y1:
#         raise ValueError("margin too large for arena")

#     pts = []
#     for i in range(lanes + 1):
#         f = i / lanes
#         cheb = 0.5 * (1.0 - math.cos(math.pi * f))   # dense near 0 and 1
#         g = (1.0 - edge_bias) * f + edge_bias * cheb
#         y = y0 + (y1 - y0) * g
#         ends = (x0, x1) if i % 2 == 0 else (x1, x0)
#         pts.extend((x, y) for x in ends)
#     return pts


# def main():
#     ap = argparse.ArgumentParser(description=__doc__,
#                                  formatter_class=argparse.RawDescriptionHelpFormatter)
#     ap.add_argument("--drone-id", type=int, required=True)
#     ap.add_argument("--config", default=str(Path(__file__).with_name("setup.yaml")))
#     ap.add_argument("--lanes", type=int, default=6, help="Number of sweep lanes")
#     ap.add_argument("--edge-bias", type=float, default=0.65,
#                     help="0=uniform y-spacing, 1=cluster lanes at borders")
#     ap.add_argument("--margin", type=float, default=0.4,
#                     help="Distance from arena border to outer lanes (m)")
#     ap.add_argument("--expected-tags", type=int, default=0,
#                     help="If >0, finish early once this many tags are localised")
#     ap.add_argument("--takeoff", action="store_true", help="Send CMD_START first")
#     ap.add_argument("--takeoff-wait", type=float, default=8.0)
#     ap.add_argument("--confirm", action="store_true",
#                     help="Require typing GO-<drone id> before any flight command")
#     ap.add_argument("--tag-log", default="registered_tags.csv")
#     ap.add_argument("--telem-port", type=int, default=5005)
#     ap.add_argument("--cmd-port", type=int, default=5006)
#     ap.add_argument("--dry-run", action="store_true",
#                     help="Print the waypoint plan and exit (no connection)")
#     args = ap.parse_args()

#     cfg = yaml.safe_load(Path(args.config).read_text())
#     arena = cfg["arena"]
#     nav_tag_ids = {int(t) for t in cfg.get("nav_tags", {})}  # landmarks: never hover
#     log.info("Nav (landmark) tags excluded from hover: %s", sorted(nav_tag_ids) or "none")

#     sweep = generate_variable_lawnmower(
#         arena["min_x"], arena["max_x"], arena["min_y"], arena["max_y"],
#         args.margin, args.lanes, args.edge_bias)
#     # Waypoints are OFFSETS from the start-hover point (same convention as the
#     # CSV path: prepend (0,0)). Absolute goal = sampled origin + offset.
#     waypoints = [(0.0, 0.0)] + sweep

#     log.info("Lawnmower: %d lanes, edge-bias %.2f, margin %.2fm → %d waypoints",
#              args.lanes, args.edge_bias, args.margin, len(waypoints))
#     lane_ys = sorted({round(y, 2) for _, y in sweep})
#     gaps = [round(b - a, 2) for a, b in zip(lane_ys, lane_ys[1:])]
#     log.info("Lane y-positions: %s", lane_ys)
#     log.info("Lane gaps (border→middle→border): %s", gaps)

#     if args.dry_run:
#         for i, (x, y) in enumerate(waypoints):
#             log.info("  wp %2d  offset (%.2f, %.2f)", i, x, y)
#         return

#     # -------- Shared state --------
#     latest = {"pkt": None, "received_at": 0.0}
#     registry = {}                 # tag_id -> survey entry dict
#     registry_lock = threading.Lock()
#     resolved = set()              # tag ids we are done reading
#     in_view = {}                  # tag_id -> monotonic time last seen in 5008 stream
#     in_view_lock = threading.Lock()
#     events = []                   # (ts_str, text) mission event log

#     def stamp():
#         t = time.time()
#         return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int((t % 1) * 1000):03d}"

#     def event(text):
#         s = stamp()
#         events.append((s, text))
#         log.info("EVENT [%s] %s", s, text)

#     # -------- Survey callback (5005): localisation = resolved --------
#     def on_survey(did, entries):
#         if did != args.drone_id:
#             return
#         with registry_lock:
#             for e in entries:
#                 tid = e["tag_id"]
#                 old = registry.get(tid)
#                 registry[tid] = e
#                 if e["localized"] and (old is None or not old["localized"]):
#                     event(f"TAG {tid} LOCALISED  NED x={e['x']:.2f} y={e['y']:.2f} "
#                           f"z={e['z']:.2f}  obs={e['observations']}")
#                     resolved.add(tid)
#                 elif old is None:
#                     event(f"TAG {tid} ID-confirmed (pose pending)  obs={e['observations']}")

#     def on_telemetry(pkt, _ip):
#         if pkt.drone_id == args.drone_id:
#             latest["pkt"] = pkt
#             latest["received_at"] = time.monotonic()

#     # -------- 5008 live stream: early "tag in view" signal (optional) --------
#     stop_evt = threading.Event()

#     def live_listener(sock):
#         while not stop_evt.is_set():
#             try:
#                 data, _ = sock.recvfrom(4096)
#             except socket.timeout:
#                 continue
#             except OSError:
#                 break
#             pkt = parse_at_debug(data)
#             if pkt is None or pkt.drone_id != args.drone_id:
#                 continue
#             now = time.monotonic()
#             with in_view_lock:
#                 for d in pkt.detections:
#                     if (d.hamming <= AT_ID_MAX_HAMMING and d.margin > AT_ID_MIN_MARGIN
#                             and d.id not in nav_tag_ids):
#                         in_view[d.id] = now

#     live_sock = None
#     try:
#         live_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         live_sock.bind(("", AT_DEBUG_PORT))
#         live_sock.settimeout(0.2)
#         threading.Thread(target=live_listener, args=(live_sock,), daemon=True).start()
#         log.info("Live tag stream bound on UDP %d (early hover trigger active)", AT_DEBUG_PORT)
#     except OSError as exc:
#         log.warning("UDP %d unavailable (%s) — hover triggers on survey only, "
#                     "which is ~3 frames slower. Close tag_debug.py/tag_stream.py "
#                     "to enable the fast trigger.", AT_DEBUG_PORT, exc)

#     def unresolved_tag_in_view():
#         """Return a target tag id currently in view and not yet resolved, else None."""
#         now = time.monotonic()
#         # Fast path: 5008 raw detections within the last 0.5 s.
#         with in_view_lock:
#             for tid, seen in in_view.items():
#                 if tid not in resolved and now - seen < 0.5:
#                     return tid
#         # Fallback: survey has it ID-confirmed but not localised.
#         with registry_lock:
#             for tid, e in registry.items():
#                 if tid not in resolved and not e["localized"] and tid not in nav_tag_ids:
#                     return tid
#         return None

#     # -------- Comms --------
#     comms = CommsNode(listen_port=args.telem_port, cmd_port=args.cmd_port,
#                       config_path=args.config if Path(args.config).exists() else None)
#     comms.on_telemetry(on_telemetry)
#     if hasattr(comms, "on_survey"):
#         comms.on_survey(on_survey)
#     comms.start()

#     def fresh():
#         return (latest["pkt"] is not None
#                 and time.monotonic() - latest["received_at"] < TELEM_STALE_S)

#     def wait_fresh(timeout):
#         end = time.monotonic() + timeout
#         while time.monotonic() < end:
#             if fresh():
#                 return True
#             time.sleep(0.05)
#         return False

#     def goto(gx, gy):
#         comms.send_command(args.drone_id, CommandPacket(CMD_GOTO, goal_x=gx, goal_y=gy))

#     def save_tags():
#         with registry_lock:
#             rows = [dict(drone_id=args.drone_id, **registry[k]) for k in sorted(registry)]
#         p = Path(args.tag_log)
#         p.parent.mkdir(parents=True, exist_ok=True)
#         with p.open("w", newline="") as f:
#             w = csv.DictWriter(f, fieldnames=["drone_id", "tag_id", "localized",
#                                               "observations", "x", "y", "z"])
#             w.writeheader()
#             w.writerows(rows)

#     flying = not args.takeoff
#     try:
#         if not wait_fresh(30.0):
#             raise RuntimeError(f"No telemetry from drone {args.drone_id}")

#         if args.confirm:
#             if input(f"Type GO-{args.drone_id} to fly: ").strip() != f"GO-{args.drone_id}":
#                 log.info("cancelled")
#                 return

#         if args.takeoff:
#             event(f"CMD_START (autonomous takeoff) → drone {args.drone_id}")
#             comms.send_command(args.drone_id, CommandPacket(CMD_START))
#             flying = True
#             time.sleep(args.takeoff_wait)

#         if not wait_fresh(3.0):
#             raise RuntimeError("No fresh position before starting sweep")
#         origin = (latest["pkt"].ned_x, latest["pkt"].ned_y)
#         event(f"sweep start — origin ({origin[0]:.2f}, {origin[1]:.2f})")

#         # -------- Walk the lawnmower, hovering on tags --------
#         for idx, (ox, oy) in enumerate(waypoints):
#             gx, gy = origin[0] + ox, origin[1] + oy
#             log.info("→ waypoint %d/%d  goal (%.2f, %.2f)", idx, len(waypoints) - 1, gx, gy)
#             goto(gx, gy)

#             while True:
#                 if not fresh():
#                     raise RuntimeError("Telemetry stale mid-sweep")
#                 p = latest["pkt"]

#                 # Early finish if we already have all expected tags
#                 if args.expected_tags > 0:
#                     with registry_lock:
#                         n_loc = sum(1 for e in registry.values() if e["localized"])
#                     if n_loc >= args.expected_tags:
#                         event(f"all {args.expected_tags} expected tags localised — ending sweep")
#                         raise StopIteration

#                 # Tag-triggered hover
#                 tid = unresolved_tag_in_view()
#                 if tid is not None:
#                     event(f"TAG {tid} in view — HOVER to read")
#                     comms.send_command(args.drone_id, CommandPacket(CMD_HOLD))
#                     hover_start = time.monotonic()
#                     time.sleep(HOVER_SETTLE_S)
#                     while time.monotonic() - hover_start < HOVER_MAX_S:
#                         if tid in resolved:
#                             break
#                         time.sleep(0.1)
#                     if tid not in resolved:
#                         resolved.add(tid)   # timed out → partial credit, stop re-hovering
#                         event(f"TAG {tid} hover timeout ({HOVER_MAX_S:.0f}s) — "
#                               f"ID logged, pose may be missing")
#                     else:
#                         event(f"TAG {tid} read — resuming sweep")
#                     goto(gx, gy)            # resume the current leg
#                     time.sleep(RESUME_GRACE_S)
#                     continue

#                 # Arrived at this waypoint?
#                 if math.hypot(p.ned_x - gx, p.ned_y - gy) < ARRIVE_RADIUS_M \
#                         or p.nav_state == NAV_ARRIVED:
#                     break
#                 time.sleep(0.2)

#         event("sweep complete")

#     except StopIteration:
#         event("sweep ended early")
#     except (Exception, KeyboardInterrupt) as exc:
#         log.error("aborted: %s", exc)
#         if flying:
#             for _ in range(3):
#                 comms.send_command(args.drone_id, CommandPacket(CMD_LAND))
#         comms.stop()
#         if registry:
#             save_tags()
#         _report(events, registry, registry_lock, args)
#         raise

#     # -------- Return to start, land --------
#     try:
#         if flying:
#             event(f"returning to start ({origin[0]:.2f}, {origin[1]:.2f})")
#             goto(origin[0], origin[1])
#             t_ret = time.monotonic()
#             while time.monotonic() - t_ret < RETURN_TIMEOUT_S:
#                 if fresh():
#                     p = latest["pkt"]
#                     if math.hypot(p.ned_x - origin[0], p.ned_y - origin[1]) < RETURN_RADIUS_M \
#                             or p.nav_state == NAV_ARRIVED:
#                         event("returned to start")
#                         break
#                 if (time.monotonic() - t_ret) % 4.0 < 0.2:
#                     goto(origin[0], origin[1])
#                 time.sleep(0.2)
#             else:
#                 event("return timeout — landing in place")

#             event(f"CMD_LAND → drone {args.drone_id}")
#             for _ in range(3):
#                 comms.send_command(args.drone_id, CommandPacket(CMD_LAND))
#             time.sleep(2.0)
#     finally:
#         stop_evt.set()
#         comms.stop()
#         if live_sock is not None:
#             live_sock.close()
#         if registry:
#             save_tags()
#             log.info("Tag coordinates saved to %s", args.tag_log)
#         _report(events, registry, registry_lock, args)


# def _report(events, registry, registry_lock, args):
#     log.info("")
#     log.info("=" * 60)
#     log.info("MISSION REPORT — drone %d", args.drone_id)
#     log.info("=" * 60)
#     with registry_lock:
#         tags = dict(registry)
#     log.info("Tags detected: %d | localised: %d",
#              len(tags), sum(1 for e in tags.values() if e["localized"]))
#     for tid in sorted(tags):
#         e = tags[tid]
#         if e["localized"]:
#             log.info("  tag %2d  NED (%.2f, %.2f, %.2f)  obs=%d",
#                      tid, e["x"], e["y"], e["z"], e["observations"])
#         else:
#             log.info("  tag %2d  (id confirmed, no pose)  obs=%d", tid, e["observations"])
#     log.info("-" * 60)
#     log.info("Event timeline:")
#     for s, text in events:
#         log.info("  [%s] %s", s, text)
#     log.info("=" * 60)


# if __name__ == "__main__":
#     main()
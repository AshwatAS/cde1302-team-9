# #!/usr/bin/env python3
# """Generate a variable-spacing lawnmower trajectory CSV for send_trajectory.py.

# Lanes are clustered toward the y-borders (cosine distribution) and spread out
# in the middle, because tags are usually placed near borders. This is the SAME
# output format as generate_lawnmower_trajectory.py — t,x,y,z, quintic rest-to-
# rest legs — so send_trajectory.py flies it unchanged.

# Coordinates are offsets from the hover point at playback start (route begins at
# 0,0). All horizontal axes are PX4 NED. z=0 holds the start altitude.

# Example:
#     python generate_lawnmower_variable.py --config setup.yaml \
#         --lanes 7 --edge-bias 0.7 --margin 0.4 \
#         --speed 0.15 --output trajectory/lawnmower_edge.csv
# """

# import argparse
# import csv
# import math
# from pathlib import Path

# import yaml

# from protocol import TRAJ_MAX_PTS


# def variable_lanes(min_x, max_x, min_y, max_y, margin, lanes, edge_bias):
#     """Sweep points in arena coords, lanes along x, y clustered at borders."""
#     if not (0.0 <= edge_bias <= 1.0):
#         raise ValueError("--edge-bias must be in [0, 1]")
#     if lanes < 1:
#         raise ValueError("--lanes must be >= 1")
#     x0, x1 = min_x + margin, max_x - margin
#     y0, y1 = min_y + margin, max_y - margin
#     if x0 >= x1 or y0 >= y1:
#         raise ValueError("--margin too large for arena")
#     pts = []
#     for i in range(lanes + 1):
#         f = i / lanes
#         cheb = 0.5 * (1.0 - math.cos(math.pi * f))     # dense near 0 and 1
#         g = (1.0 - edge_bias) * f + edge_bias * cheb
#         y = y0 + (y1 - y0) * g
#         ends = (x0, x1) if i % 2 == 0 else (x1, x0)
#         pts.extend((x, y) for x in ends)
#     return pts


# def make_trajectory(points, speed, acceleration, dt, hold):
#     """Quintic rest-to-rest legs through the given points, starting at (0,0)."""
#     if not all(math.isfinite(v) and v > 0 for v in (speed, acceleration, dt)):
#         raise ValueError("speed, acceleration and dt must be positive and finite")
#     route = [(0.0, 0.0)] + points
#     rows = [(0.0, 0.0, 0.0, 0.0)]
#     for start, end in zip(route, route[1:]):
#         dx, dy = end[0] - start[0], end[1] - start[1]
#         distance = math.hypot(dx, dy)
#         if distance == 0:
#             continue
#         # q(u)=10u^3-15u^4+6u^5: max q'=1.875, max |q''|=10/sqrt(3).
#         duration = max(1.875 * distance / speed,
#                        math.sqrt((10 / math.sqrt(3)) * distance / acceleration))
#         steps = max(2, math.ceil(duration / dt))
#         if len(rows) + steps > TRAJ_MAX_PTS:
#             raise ValueError(
#                 f"Route needs > {TRAJ_MAX_PTS} samples. Fewer lanes, smaller "
#                 f"area, higher --speed, or a larger --dt.")
#         for step in range(1, steps + 1):
#             u = step / steps
#             q = u**3 * (10 + u * (-15 + 6 * u))
#             rows.append((len(rows) * dt, start[0] + q * dx, start[1] + q * dy, 0.0))
#     for _ in range(math.ceil(hold / dt)):
#         rows.append((len(rows) * dt, *rows[-1][1:]))
#     return rows


# def main():
#     ap = argparse.ArgumentParser(description=__doc__,
#                                  formatter_class=argparse.RawDescriptionHelpFormatter)
#     ap.add_argument("--config", default=str(Path(__file__).with_name("setup.yaml")))
#     ap.add_argument("--lanes", type=int, default=7)
#     ap.add_argument("--edge-bias", type=float, default=0.7,
#                     help="0=uniform y-spacing, 1=lanes clustered at borders")
#     ap.add_argument("--margin", type=float, default=0.4,
#                     help="Distance from arena border to outer lanes (m)")
#     ap.add_argument("--speed", type=float, default=0.15, help="Peak speed m/s")
#     ap.add_argument("--acceleration", type=float, default=0.2)
#     ap.add_argument("--dt", type=float, default=0.2, help="Sample interval s (match sender --dt)")
#     ap.add_argument("--end-hold", type=float, default=2.0)
#     ap.add_argument("--output", required=True)
#     args = ap.parse_args()
#     if not 0.001 <= args.dt <= 1:
#         ap.error("--dt must be 0.001 to 1 s")

#     arena = yaml.safe_load(Path(args.config).read_text())["arena"]
#     pts = variable_lanes(arena["min_x"], arena["max_x"],
#                          arena["min_y"], arena["max_y"],
#                          args.margin, args.lanes, args.edge_bias)
#     try:
#         rows = make_trajectory(pts, args.speed, args.acceleration, args.dt, args.end_hold)
#     except ValueError as exc:
#         ap.error(str(exc))

#     out = Path(args.output)
#     out.parent.mkdir(parents=True, exist_ok=True)
#     with out.open("w", newline="") as f:
#         w = csv.writer(f)
#         w.writerow(["t", "x", "y", "z"])
#         w.writerows(rows)

#     lane_ys = sorted({round(y, 2) for _, y in pts})
#     gaps = [round(b - a, 2) for a, b in zip(lane_ys, lane_ys[1:])]
#     print(f"Saved {out}: {len(rows)} points, {rows[-1][0]:.1f} s, dt={args.dt:g}")
#     print(f"Lanes at y = {lane_ys}")
#     print(f"Lane gaps (border→middle→border) = {gaps}")
#     print(f"Fly with:  python send_trajectory.py --drone-id <id> --takeoff "
#           f"--dt {args.dt:g} --trajectory {out}")


# if __name__ == "__main__":
#     main()




#!/usr/bin/env python3
"""Generate a lawnmower trajectory CSV for send_trajectory.py, with:

  1. Variable lane spacing   — lanes clustered at the y-borders (where tags
     usually sit), wider in the middle.  --edge-bias controls this.
  2. Tag-aware speed profile — the drone slows to --slow-speed within
     --slow-radius of any supplied tag coordinate and cruises at --fast-speed
     elsewhere, with a smooth accel-limited ramp between the two.  This lets
     it dwell over tags long enough to read them without stopping, and cover
     empty space quickly.

Output is the same t,x,y,z format send_trajectory.py already flies — no
firmware change, no sender change.

COORDINATE FRAME (important): tag coordinates must be given in the SAME frame
as the trajectory — arena metres measured from the hover/launch point (the
(x, y) the drone is at when directly over the tag).  The route starts at
(0, 0) = hover point; z is held at the start altitude.

Examples
--------
  # tags inline (x,y pairs separated by ';')
  python generate_lawnmower_variable.py --config setup.yaml \
      --tags "1.0,-1.4; 3.6,-1.3; 2.0,1.4; 3.8,1.2" \
      --fast-speed 0.4 --slow-speed 0.1 --slow-radius 0.7 \
      --lanes 7 --edge-bias 0.7 --output ../trajectory/lawnmower_tags.csv

  # tags from a CSV with x,y columns (header optional)
  python generate_lawnmower_variable.py --config setup.yaml \
      --tags-file tags.csv --output ../trajectory/lawnmower_tags.csv

  # no tags -> uniform --fast-speed everywhere (same as before)
  python generate_lawnmower_variable.py --config setup.yaml \
      --fast-speed 0.15 --output ../trajectory/lawnmower.csv
"""

import argparse
import csv
import math
from pathlib import Path

import yaml

from protocol import TRAJ_MAX_PTS


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
def variable_lanes(min_x, max_x, min_y, max_y, margin, lanes, edge_bias):
    """Sweep points in arena coords, lanes along x, y clustered at borders."""
    if not (0.0 <= edge_bias <= 1.0):
        raise ValueError("--edge-bias must be in [0, 1]")
    if lanes < 1:
        raise ValueError("--lanes must be >= 1")
    x0, x1 = min_x + margin, max_x - margin
    y0, y1 = min_y + margin, max_y - margin
    if x0 >= x1 or y0 >= y1:
        raise ValueError("--margin too large for arena")
    pts = []
    for i in range(lanes + 1):
        f = i / lanes
        cheb = 0.5 * (1.0 - math.cos(math.pi * f))     # dense near 0 and 1
        g = (1.0 - edge_bias) * f + edge_bias * cheb
        y = y0 + (y1 - y0) * g
        ends = (x0, x1) if i % 2 == 0 else (x1, x0)
        pts.extend((x, y) for x in ends)
    return pts


def in_zone(pt, tags, radius):
    """True if pt is within radius of any tag coordinate."""
    return any(math.hypot(pt[0] - tx, pt[1] - ty) <= radius for tx, ty in tags)


def split_legs(route, tags, radius):
    """Split each polyline leg where it crosses a tag-zone boundary. Returns a
    list of (start, end, slow) sub-segments; `slow` True inside a tag zone."""
    segs = []
    for a, b in zip(route, route[1:]):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        if length == 0:
            continue
        n = max(2, int(length / 0.02))           # 2 cm classification resolution
        params = [i / n for i in range(n + 1)]
        pts = [(a[0] + p * (b[0] - a[0]), a[1] + p * (b[1] - a[1])) for p in params]
        flags = [in_zone(pt, tags, radius) for pt in pts] if tags else [False] * len(pts)
        # node params: leg ends + every flag transition midpoint
        nodes = [0.0]
        for i in range(1, len(flags)):
            if flags[i] != flags[i - 1]:
                nodes.append((params[i] + params[i - 1]) / 2)
        nodes.append(1.0)
        for p0, p1 in zip(nodes, nodes[1:]):
            mid = (p0 + p1) / 2
            s = (a[0] + p0 * (b[0] - a[0]), a[1] + p0 * (b[1] - a[1]))
            e = (a[0] + p1 * (b[0] - a[0]), a[1] + p1 * (b[1] - a[1]))
            slow = in_zone((a[0] + mid * (b[0] - a[0]),
                            a[1] + mid * (b[1] - a[1])), tags, radius) if tags else False
            segs.append((s, e, slow))
    return segs


def make_trajectory(segs, fast, slow, accel, dt, dwell):
    """Quintic rest-to-rest per sub-segment (smooth, stops at every node, so
    the velocity vector never flips at speed). Peak speed is `slow` inside a
    tag zone, `fast` elsewhere. Optional dwell holds at the end of each in-zone
    segment for extra read time."""
    rows = [(0.0, segs[0][0][0], segs[0][0][1], 0.0)]
    for s, e, is_slow in segs:
        dx, dy = e[0] - s[0], e[1] - s[1]
        d = math.hypot(dx, dy)
        if d < 1e-6:
            continue
        vpk = slow if is_slow else fast
        # q(u)=10u^3-15u^4+6u^5: peak |q'|=1.875, peak |q''|=10/sqrt(3).
        duration = max(1.875 * d / vpk, math.sqrt((10 / math.sqrt(3)) * d / accel))
        steps = max(2, math.ceil(duration / dt))
        for step in range(1, steps + 1):
            u = step / steps
            q = u ** 3 * (10 + u * (-15 + 6 * u))
            rows.append((rows[-1][0] + dt, s[0] + q * dx, s[1] + q * dy, 0.0))
        if is_slow and dwell > 0:
            for _ in range(math.ceil(dwell / dt)):
                rows.append((rows[-1][0] + dt, e[0], e[1], 0.0))
    return rows


# ---------------------------------------------------------------------------
# Tag parsing
# ---------------------------------------------------------------------------
def parse_tags_inline(spec):
    tags = []
    for chunk in spec.replace("\n", ";").split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        x, y = (float(v) for v in chunk.split(","))
        tags.append((x, y))
    return tags


def parse_tags_file(path):
    tags = []
    for row in csv.reader(Path(path).read_text().splitlines()):
        if len(row) < 2:
            continue
        try:
            tags.append((float(row[0]), float(row[1])))
        except ValueError:
            continue                              # header or blank line
    return tags


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(Path(__file__).with_name("setup.yaml")))
    ap.add_argument("--output", required=True)
    # geometry
    ap.add_argument("--lanes", type=int, default=7)
    ap.add_argument("--edge-bias", type=float, default=0.7,
                    help="0=uniform y-spacing, 1=lanes clustered at borders")
    ap.add_argument("--margin", type=float, default=0.4,
                    help="Distance from arena border to outer lanes (m)")
    # tags + speed
    ap.add_argument("--tags", default="",
                    help='Tag coords, arena frame: "x,y; x,y; ..."')
    ap.add_argument("--tags-file", default="",
                    help="CSV file with x,y columns (header optional)")
    ap.add_argument("--fast-speed", type=float, default=0.4, help="Cruise speed m/s")
    ap.add_argument("--slow-speed", type=float, default=0.1, help="Speed near tags m/s")
    ap.add_argument("--slow-radius", type=float, default=0.7,
                    help="Within this distance of a tag, fly at --slow-speed (m)")
    ap.add_argument("--dwell", type=float, default=0.0,
                    help="Extra hover time (s) at the end of each tag zone to read")
    ap.add_argument("--acceleration", type=float, default=0.25, help="m/s^2 cap")
    ap.add_argument("--dt", type=float, default=0.2,
                    help="Sample interval s (must match sender --dt)")
    ap.add_argument("--end-hold", type=float, default=2.0)
    args = ap.parse_args()

    if not 0.001 <= args.dt <= 1:
        ap.error("--dt must be 0.001 to 1 s")
    if args.slow_speed <= 0 or args.fast_speed <= 0 or args.acceleration <= 0:
        ap.error("speeds and acceleration must be positive")
    if args.slow_speed > args.fast_speed:
        ap.error("--slow-speed must be <= --fast-speed")

    tags = parse_tags_inline(args.tags)
    if args.tags_file:
        tags += parse_tags_file(args.tags_file)

    arena = yaml.safe_load(Path(args.config).read_text())["arena"]
    lanes = variable_lanes(arena["min_x"], arena["max_x"],
                           arena["min_y"], arena["max_y"],
                           args.margin, args.lanes, args.edge_bias)
    route = [(0.0, 0.0)] + lanes

    segs = split_legs(route, tags, args.slow_radius)
    rows = make_trajectory(segs, args.fast_speed, args.slow_speed,
                           args.acceleration, args.dt, args.dwell)

    for _ in range(math.ceil(args.end_hold / args.dt)):
        rows.append((rows[-1][0] + args.dt, rows[-1][1], rows[-1][2], 0.0))

    if len(rows) > TRAJ_MAX_PTS:
        ap.error(f"{len(rows)} samples > firmware cap {TRAJ_MAX_PTS}. Raise "
                 f"--fast-speed, --dt, or --slow-speed; or fewer --lanes.")

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z"])
        w.writerows((round(t, 3), round(x, 4), round(y, 4), round(z, 4))
                    for t, x, y, z in rows)

    # ---- report ----
    lane_ys = sorted({round(y, 2) for _, y in lanes})
    print(f"Saved {out}")
    print(f"  {len(rows)} samples (cap {TRAJ_MAX_PTS}), {rows[-1][0]:.1f} s, dt={args.dt:g}")
    print(f"  lanes at y = {lane_ys}")
    if tags:
        # per-tag: closest approach distance + min speed while inside its zone
        print(f"  {len(tags)} tag zone(s), slow={args.slow_speed} fast={args.fast_speed} "
              f"radius={args.slow_radius} dwell={args.dwell}")
        for ti, (tx, ty) in enumerate(tags):
            dmin = min(math.hypot(x - tx, y - ty) for _, x, y, _ in rows)
            # peak speed while inside this tag's zone (should be <= slow-speed)
            near = [math.hypot(rows[i+1][1]-rows[i][1], rows[i+1][2]-rows[i][2]) / args.dt
                    for i in range(len(rows) - 1)
                    if math.hypot(rows[i][1]-tx, rows[i][2]-ty) < args.slow_radius]
            vpk = max(near) if near else float("nan")
            flag = "" if dmin <= args.slow_radius else "  <-- PATH NEVER PASSES WITHIN radius!"
            print(f"    tag {ti} ({tx:+.2f},{ty:+.2f})  closest={dmin:.2f}m  "
                  f"peak-speed-in-zone={vpk:.2f}m/s{flag}")
    else:
        print(f"  no tags given -> uniform {args.fast_speed} m/s")
    print(f"Fly with:  python send_trajectory.py --drone-id <id> --takeoff "
          f"--dt {args.dt:g} --trajectory {out} --return-start")


if __name__ == "__main__":
    main()

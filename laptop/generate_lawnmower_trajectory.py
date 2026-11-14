#!/usr/bin/env python3
"""Generate t,x,y,z for send_trajectory.py; coordinates relative to start hover.

No connection or flight commands. All horizontal axes remain PX4 NED axes.
Each straight leg uses a quintic rest-to-rest time profile. z=0 preserves
the altitude at playback start. This is an obstacle-free rectangular route.
"""

import argparse
import csv
import math
from pathlib import Path

import yaml

from lawnmower import generate_lawnmower
from protocol import TRAJ_MAX_PTS


def make_trajectory(bounds, spacing, margin, speed, acceleration, dt, hold=2.0):
    """Return uniformly timed (t,x,y,z) rows with all corners included."""
    if not all(math.isfinite(v) and v > 0 for v in (speed, acceleration, dt)):
        raise ValueError("Speed, acceleration and dt must be positive and finite")
    if not math.isfinite(hold) or hold < 0:
        raise ValueError("End hold must be finite and non-negative")
    points = [(0.0, 0.0)] + generate_lawnmower(*bounds, spacing, margin)
    rows = [(0.0, 0.0, 0.0, 0.0)]
    for start, end in zip(points, points[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        distance = math.hypot(dx, dy)
        if distance == 0:
            continue
        # q(u)=10u^3-15u^4+6u^5 has max q'=1.875 and max |q''|=10/sqrt(3).
        duration = max(1.875 * distance / speed,
                       math.sqrt((10 / math.sqrt(3)) * distance / acceleration))
        steps = max(2, math.ceil(duration / dt))
        if len(rows) + steps > TRAJ_MAX_PTS:
            raise ValueError(
                f"Route exceeds firmware limit of {TRAJ_MAX_PTS} samples. "
                "Use a smaller area, fewer lanes, or a larger --dt (up to 1 s)."
            )
        for step in range(1, steps + 1):
            u = step / steps
            q = u**3 * (10 + u * (-15 + 6 * u))
            rows.append((len(rows) * dt, start[0] + q * dx, start[1] + q * dy, 0.0))
    for _ in range(math.ceil(hold / dt)):
        rows.append((len(rows) * dt, *rows[-1][1:]))
    if len(rows) > TRAJ_MAX_PTS:
        raise ValueError("End hold exceeds sample capacity; shorten the route or increase --dt")
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(Path(__file__).with_name("setup.yaml")))
    parser.add_argument("--length", type=float, help="Override arena with x=0..length; requires --width")
    parser.add_argument("--width", type=float, help="Override arena with y=0..width; requires --length")
    parser.add_argument("--lane-spacing", type=float, required=True)
    parser.add_argument("--margin", type=float, required=True)
    parser.add_argument("--speed", type=float, default=0.15, help="Peak planned speed in m/s (default .15)")
    parser.add_argument("--acceleration", type=float, default=0.2, help="Peak planned acceleration in m/s^2")
    parser.add_argument("--dt", type=float, default=0.05, help="Sample interval in seconds; match sender --dt")
    parser.add_argument("--end-hold", type=float, default=2.0)
    parser.add_argument("--output", required=True, help="Destination CSV")
    args = parser.parse_args()
    if not math.isfinite(args.dt) or not 0.001 <= args.dt <= 1:
        parser.error("--dt must be 0.001 to 1 second")
    dt = round(args.dt * 1000) / 1000
    if (args.length is None) != (args.width is None):
        parser.error("Specify both --length and --width")
    if args.length is not None:
        bounds = (0.0, args.length, 0.0, args.width)
    else:
        arena = yaml.safe_load(Path(args.config).read_text())["arena"]
        bounds = tuple(arena[k] for k in ("min_x", "max_x", "min_y", "max_y"))
    try:
        rows = make_trajectory(bounds, args.lane_spacing, args.margin,
                               args.speed, args.acceleration, dt, args.end_hold)
    except ValueError as exc:
        parser.error(str(exc))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["t", "x", "y", "z"])
        writer.writerows(rows)
    print(f"Saved {output}: {len(rows)} points, {rows[-1][0]:.2f} seconds, dt={dt:g}")
    print("Origin: hover position at trajectory start. Axes: +x north, +y east.")
    print("z=0 offset holds start altitude. Corners use planned rest-to-rest motion.")
    print(f"Use send_trajectory.py with --dt {dt:g} and --trajectory {output}")


if __name__ == "__main__":
    main()

"""Rectangular, obstacle-free coverage in arena coordinates (metres)."""

import math


def generate_lawnmower(min_x, max_x, min_y, max_y, lane_spacing, margin):
    """Sweep along x, starting at the inset minimum-x/minimum-y corner.

    Include both outer lanes. Actual spacing is at most lane_spacing.
    Margin is the distance from the arena boundary to the outer flight lines.
    This generates positions only; it does not guarantee camera coverage.
    """
    values = (min_x, max_x, min_y, max_y, lane_spacing, margin)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Bounds, spacing and margin must be finite")
    if lane_spacing <= 0 or margin < 0:
        raise ValueError("Spacing must be positive and margin non-negative")
    x0, x1 = min_x + margin, max_x - margin
    y0, y1 = min_y + margin, max_y - margin
    if x0 >= x1 or y0 >= y1:
        raise ValueError("Bounds and margin must leave a non-empty rectangle")
    intervals = math.ceil((y1 - y0) / lane_spacing)
    if intervals > 10000:
        raise ValueError("Too many lanes; increase lane spacing")
    points = []
    for lane in range(intervals + 1):
        y = y0 + (y1 - y0) * lane / intervals
        ends = (x0, x1) if lane % 2 == 0 else (x1, x0)
        points.extend((x, y) for x in ends)
    return points

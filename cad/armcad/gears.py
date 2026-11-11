"""Involute spur-gear outline used by the gripper fingers."""
from __future__ import annotations

import math
from typing import List, Tuple

Point2 = Tuple[float, float]


def _involute(angle: float) -> float:
    return math.tan(angle) - angle


def gear_outline(module: float, teeth: int, backlash: float = 0.25,
                 pressure_deg: float = 20.0, phase_deg: float = 0.0,
                 flank_points: int = 8) -> List[Point2]:
    """Closed polygon of a spur gear centred on the origin.

    With ``phase_deg = 0`` a tooth is centred on the +X axis.
    """
    alpha = math.radians(pressure_deg)
    pitch_r = module * teeth / 2.0
    base_r = pitch_r * math.cos(alpha)
    tip_r = pitch_r + module
    root_r = pitch_r - 1.25 * module
    half_thick = math.pi / (2 * teeth) - backlash / (2 * pitch_r)

    def half_angle(radius: float) -> float:
        """Angular half-thickness of a tooth at ``radius``."""
        if radius <= base_r:
            return half_thick + _involute(alpha)
        return half_thick + _involute(alpha) - _involute(math.acos(base_r / radius))

    start_r = max(base_r, root_r)
    radii = [start_r + (tip_r - start_r) * i / (flank_points - 1)
             for i in range(flank_points)]
    if root_r < base_r:
        radii.insert(0, root_r)

    outline: List[Point2] = []
    for k in range(teeth):
        centre = math.radians(phase_deg) + 2 * math.pi * k / teeth
        trailing = [(r, centre - half_angle(r)) for r in radii]
        leading = [(r, centre + half_angle(r)) for r in reversed(radii)]
        outline += [(r * math.cos(a), r * math.sin(a)) for r, a in trailing + leading]
    return outline

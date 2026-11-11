"""Joint conventions, forward/inverse kinematics and the transfer trajectory.

Angles are degrees.  Zero pose: upper arm vertical, forearm horizontal pointing
away from the turret axis (to -Y when the turret is at 0), tool pointing down,
fingers parallel.

    turret       about +Z   positive turns the arm towards +X
    shoulder     about +X   positive leans the upper arm forward
    elbow        about +X   positive lowers the forearm tip
    wrist_pitch  about +X   -(shoulder + elbow) keeps the tool vertical
    wrist_roll   about tool  -turret keeps the payload heading fixed
    grip         jaw opening in mm, measured between the rubber pads
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Dict, List, Tuple

from .layout import ArmLayout


@dataclass(frozen=True)
class JointState:
    turret: float = 0.0
    shoulder: float = 0.0
    elbow: float = 0.0
    wrist_pitch: float = 0.0
    wrist_roll: float = 0.0
    grip: float = 38.0

    def with_(self, **changes) -> "JointState":
        return replace(self, **changes)


#: Mechanical travel the design is checked for (min, max).
JOINT_LIMITS: Dict[str, Tuple[float, float]] = {
    "turret": (-135.0, 135.0),
    "shoulder": (-30.0, 90.0),
    "elbow": (-90.0, 45.0),
    "wrist_pitch": (-90.0, 30.0),
    "wrist_roll": (-90.0, 90.0),
    "grip": (26.0, 58.0),
}


class UnreachableError(ValueError):
    pass


class ArmKinematics:
    """Closed-form kinematics for a vertical tool (pick-and-place) posture."""

    def __init__(self, layout: ArmLayout):
        self.layout = layout

    def finger_angle(self, grip_width: float) -> float:
        """Finger rotation (deg) from parallel for a given pad-to-pad opening."""
        lay = self.layout
        half_travel = (grip_width - lay.grip_nominal_width) / 2.0
        return math.degrees(math.asin(half_travel / lay.pad_centre_below_pivot))

    def solve(self, x: float, y: float, z: float, heading: float = 0.0,
              grip: float = 38.0) -> JointState:
        """Joint angles that put the pad centre at world (x, y, z), tool vertical.

        ``heading`` is the payload yaw about world Z (0 = jaws close along Y).
        """
        lay = self.layout
        dx, dy = x - lay.base_xy[0], y - lay.base_xy[1]
        turret = math.degrees(math.atan2(dx, -dy))
        reach = math.hypot(dx, dy)
        height = z + lay.tool_length - lay.shoulder_z
        l1, l2 = lay.upper_arm_length, lay.forearm_length
        dist = math.hypot(reach, height)
        if not abs(l1 - l2) < dist < l1 + l2:
            raise UnreachableError(f"target ({x:.0f}, {y:.0f}, {z:.0f}) is out of reach")
        elevation = math.atan2(height, reach)
        at_shoulder = math.acos((l1 * l1 + dist * dist - l2 * l2) / (2 * l1 * dist))
        at_elbow = math.acos((l1 * l1 + l2 * l2 - dist * dist) / (2 * l1 * l2))
        shoulder = 90.0 - math.degrees(elevation + at_shoulder)
        forearm_drop = math.degrees(math.pi - at_elbow - elevation - at_shoulder)
        state = JointState(turret=turret, shoulder=shoulder,
                           elbow=forearm_drop - shoulder, wrist_pitch=-forearm_drop,
                           wrist_roll=heading - turret, grip=grip)
        self.assert_within_limits(state)
        return state

    def pad_centre(self, q: JointState) -> Tuple[float, float, float]:
        """Forward kinematics of the pad centre (tool assumed vertical)."""
        lay = self.layout
        s, e = math.radians(q.shoulder), math.radians(q.shoulder + q.elbow)
        reach = lay.upper_arm_length * math.sin(s) + lay.forearm_length * math.cos(e)
        height = (lay.shoulder_z + lay.upper_arm_length * math.cos(s)
                  - lay.forearm_length * math.sin(e) - lay.tool_length)
        t = math.radians(q.turret)
        return (lay.base_xy[0] + reach * math.sin(t), lay.base_xy[1] - reach * math.cos(t),
                height)

    @staticmethod
    def assert_within_limits(q: JointState) -> None:
        for name, (lo, hi) in JOINT_LIMITS.items():
            value = getattr(q, name)
            if not lo - 1e-6 <= value <= hi + 1e-6:
                raise UnreachableError(f"{name} = {value:.1f} is outside [{lo}, {hi}]")


@dataclass(frozen=True)
class TransferPlan:
    """Pick at zone A, place at zone B: the motion study of the concept model."""

    grasp_height: float = 14.0     # pad centre above the grid when gripping
    lift: float = 45.0
    apex: float = 55.0
    open_width: float = 54.0
    hold_width: float = 36.5       # slightly under the truck width: pads squeeze

    def waypoints(self, kin: ArmKinematics, steps: int = 60) -> List[JointState]:
        lay = kin.layout
        (ax, ay), (bx, by) = lay.zone_a, lay.zone_b
        cx, cy = lay.base_xy
        ang_a = math.atan2(ax - cx, -(ay - cy))
        ang_b = math.atan2(bx - cx, -(by - cy))
        radius = math.hypot(ax - cx, ay - cy)
        lift_end, slew_end = steps // 5, steps - steps // 5

        def ease(u: float) -> float:
            return 0.5 - 0.5 * math.cos(math.pi * min(max(u, 0.0), 1.0))

        states = []
        for i in range(steps + 1):
            if i <= lift_end:
                ang, z = ang_a, self.grasp_height + self.lift * ease(i / lift_end)
            elif i <= slew_end:
                u = (i - lift_end) / (slew_end - lift_end)
                ang = ang_a + (ang_b - ang_a) * ease(u)
                z = self.grasp_height + self.lift + (self.apex - self.lift) * math.sin(math.pi * u)
            else:
                u = (i - slew_end) / (steps - slew_end)
                ang, z = ang_b, self.grasp_height + self.lift * (1 - ease(u))
            x, y = cx + radius * math.sin(ang), cy - radius * math.cos(ang)
            states.append(kin.solve(x, y, z, heading=0.0, grip=self.hold_width))
        return states

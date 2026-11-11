"""Gripper: body that hangs from the roll horn and two geared fingers."""
from __future__ import annotations

import math
from typing import List

import cadquery as cq

from ..gears import gear_outline
from ..geometry import (Point2, box, cut, cyl_x, cyl_z, prism_yz, rotation, slot_xy,
                        slot_yz, union)
from .base_part import PrintedPart

GEAR_MODULE = 1.5
GEAR_TEETH = 18              # pitch diameter 27 mm = finger pivot spacing


class GripperBody(PrintedPart):
    name = "gripper_body"
    link = "gripper"
    print_note = "Print upside down (flat top plate on the bed). No supports."
    print_rotation = rotation((1, 0, 0), 180)

    top_plate = 3.0
    wall = 5.0
    y_span = (-21.0, 27.0)
    depth = 19.5

    def build(self) -> cq.Shape:
        lay, tol = self.layout, self.layout.tol
        m = lay.micro
        servo = lay.grip_servo()
        seat_x, pz = lay.grip_seat_x, lay.grip_pivot_z
        face_x = lay.finger_plane_x - 0.2
        tab_end = lay.finger_pivot_half - m.shaft_to_long_end - m.tab_extension

        body = union([
            # stops short of the finger plane so the grip horn can swing past it
            box((seat_x - self.wall, face_x - 2.0), self.y_span, (-self.top_plate, 0)),
            box((seat_x - self.wall, seat_x), self.y_span, (-self.depth, 0)),
            # pivot boss for the idler finger
            box((seat_x, face_x), (self.y_span[0], tab_end - 0.5), (-self.depth + 2, 0)),
        ])
        z = (-self.top_plate - 0.5, 0.5)
        return cut(body, [
            servo.case_cutter(open_towards="+y"),
            servo.pilot_cutter(depth=self.wall + 1, diameter=tol.m2_tap),
            cyl_x((seat_x - 1, face_x + 1), -lay.finger_pivot_half, pz, tol.m3_tap / 2),
            # roll horn: centre-screw access and one slot for two M2 screws
            cyl_z(0, 0, z, 2.6),
            slot_xy((0, 0), 90.0, lay.arm_horn.hole_r_min, lay.arm_horn.hole_r_max,
                    tol.m2_clear, z),
        ])


def _convex_hull(points: List[Point2]) -> List[Point2]:
    pts = sorted(set(points))

    def half(seq):
        out: List[Point2] = []
        for p in seq:
            while len(out) >= 2 and ((out[-1][0] - out[-2][0]) * (p[1] - out[-2][1])
                                     - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(p)
        return out[:-1]

    return half(pts) + half(reversed(pts))


class Finger(PrintedPart):
    """One gripper finger, modelled about its pivot axis.

    ``side`` is +1 for the finger on the grip-servo horn, -1 for the idler.
    """

    print_note = "Print with the flat gear face on the bed. No supports."
    print_rotation = rotation((0, 1, 0), 90)

    hub_radius = 7.5
    pad_span = (-62.0, -24.0)      # z range of the jaw plate below the pivot
    pad_x0 = -10.0
    jaw_thickness = 4.0
    horn_slot_deg = -73.3          # direction of the horn arm on the driven finger

    def __init__(self, layout, side: int):
        super().__init__(layout)
        self.side = side
        self.name = "finger_drive" if side > 0 else "finger_idler"
        self.link = "finger_a" if side > 0 else "finger_b"

    @property
    def jaw_y(self) -> tuple:
        """Pad plate y range (pivot frame) of the +1 finger."""
        lay = self.layout
        inner = lay.grip_nominal_width / 2 + lay.pad_thickness - lay.finger_pivot_half
        return (inner, inner + self.jaw_thickness)

    def build(self) -> cq.Shape:
        lay, tol = self.layout, self.layout.tol
        s = self.side
        x = (lay.finger_plane_x, lay.finger_plane_x + lay.finger_plate)
        y0, y1 = self.jaw_y

        circle = [(self.hub_radius * math.cos(math.radians(a)),
                   self.hub_radius * math.sin(math.radians(a))) for a in range(0, 360, 10)]
        # the arm meets the jaw plate above the payload, so nothing but the
        # jaw plates (and their pads) comes between the fingers
        top = self.pad_span[1]
        arm_box = [(s * y0, top), (s * y1, top), (s * y1, top + 2.0), (s * y0, top + 2.0)]
        arm = prism_yz(_convex_hull(circle + arm_box), x)

        # teeth face the other finger; the idler is half a pitch out of phase
        facing = 180.0 if s > 0 else 0.0
        phase = facing if s > 0 else facing + 180.0 / GEAR_TEETH
        gear = prism_yz(gear_outline(GEAR_MODULE, GEAR_TEETH, phase_deg=phase), x)
        reach = 30.0
        wedge = [(0.0, 0.0)] + [(reach * math.cos(math.radians(facing + a)),
                                 reach * math.sin(math.radians(facing + a)))
                                for a in (-44, -22, 0, 22, 44)]
        sector = gear.intersect(prism_yz(wedge, x))

        jaw = box((self.pad_x0, x[1]), (s * y0, s * y1), self.pad_span)
        body = union([arm, sector, jaw])

        if s > 0:
            tools = [cyl_x((x[0] - 1, x[1] + 1), 0, 0, 2.6),
                     slot_yz((0, 0), self.horn_slot_deg, lay.arm_horn.hole_r_min,
                             lay.arm_horn.hole_r_max, tol.m2_clear, (x[0] - 1, x[1] + 1))]
        else:
            tools = [cyl_x((x[0] - 1, x[1] + 1), 0, 0, tol.m3_clear / 2)]
        return cut(body, tools)

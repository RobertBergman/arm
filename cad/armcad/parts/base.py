"""Pedestal: holds the turret servo and bolts to the grid."""
from __future__ import annotations

import math

import cadquery as cq

from ..geometry import box, cut, cyl_z, union
from .base_part import PrintedPart


class Base(PrintedPart):
    name = "base"
    link = "base"
    print_note = "Print upright as modelled. No supports."

    flange_radius = 44.0
    flange_height = 6.0
    column_radius = 33.0
    rear_box = ((-16.0, 16.0), (0.0, 45.0))
    bolt_circle = 38.0
    bolt_angles = (0, 50, 130, 180, 230, 310)
    rim_inner = 31.6
    rim_height = 1.0

    def bolt_positions(self):
        return [(self.bolt_circle * math.cos(math.radians(a)),
                 self.bolt_circle * math.sin(math.radians(a))) for a in self.bolt_angles]

    def build(self) -> cq.Shape:
        lay, tol = self.layout, self.layout.tol
        top = lay.base_top_z
        servo = lay.base_servo()
        s = lay.std

        rim = cyl_z(0, 0, (top, top + self.rim_height), self.column_radius).cut(
            cyl_z(0, 0, (top - 1, top + self.rim_height + 1), self.rim_inner))
        body = union([
            cyl_z(0, 0, (0, self.flange_height), self.flange_radius),
            cyl_z(0, 0, (0, top), self.column_radius),
            box(self.rear_box[0], self.rear_box[1], (0, top)),
            rim,
        ])

        front_end = -s.shaft_to_short_end - tol.pocket
        rear_end = s.shaft_to_long_end + tol.pocket
        cable = [
            box((-3.5, 3.5), (front_end - 3.5, front_end), (-1, 14)),
            box((-3.5, 3.5), (rear_end, rear_end + 3.5), (-1, 14)),
            box((-4.0, 4.0), (rear_end, self.rear_box[1][1] + 1), (-1, 8)),
        ]
        bolts = [cyl_z(x, y, (-1, self.flange_height + 1), tol.m3_clear / 2)
                 for x, y in self.bolt_positions()]
        return cut(body, [
            servo.case_cutter(bottom_gap=lay.base_seat_z),   # open through the floor
            servo.upper_cutter(height=top),
            servo.pilot_cutter(depth=12.0, diameter=tol.m3_tap),
            *cable, *bolts,
        ])


class TurretShim(PrintedPart):
    """Thrust washer between base and turret; pick the thickness that fits."""

    link = "base"
    print_note = "Print flat. Print the whole set, use the thickest that turns freely."

    def __init__(self, layout, thickness: float):
        super().__init__(layout)
        self.thickness = thickness
        self.name = f"turret_shim_{thickness:.1f}mm".replace(".", "p")

    def build(self) -> cq.Shape:
        z0 = self.layout.base_top_z
        ring = cyl_z(0, 0, (z0, z0 + self.thickness), 31.2)
        return ring.cut(cyl_z(0, 0, (z0 - 1, z0 + self.thickness + 1), 22.0))

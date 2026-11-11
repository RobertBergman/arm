"""Wrist block: holds the wrist-pitch servo and, below it, the roll servo."""
from __future__ import annotations

import cadquery as cq

from ..geometry import box, cut, cyl_x, rotation, union
from .base_part import PrintedPart


class Wrist(PrintedPart):
    name = "wrist"
    link = "wrist"
    print_note = "Print with the solid side wall on the bed. No supports."
    print_rotation = rotation((0, 1, 0), -90)

    seat_block = 6.0          # length of the blocks the servo tabs screw into
    lower_reach = 9.0         # +x extent of the roll-servo seats

    def build(self) -> cq.Shape:
        lay, tol = self.layout, self.layout.tol
        m = lay.micro
        c = tol.pocket
        pitch, roll = lay.pitch_servo(), lay.roll_servo()
        wall_x0, wall_x1 = lay.wrist_wall_x
        seat_x = lay.wrist_seat_x
        half_w = m.width / 2 + c
        y_in = (-m.shaft_to_short_end - c, m.shaft_to_long_end + c)
        y_out = (y_in[0] - self.seat_block, y_in[1] + self.seat_block)
        z_floor = (-half_w - lay.wrist_mid_floor, -half_w)
        z_low = lay.roll_seat_z

        blocks = []
        for y in ((y_out[0], y_in[0]), (y_in[1], y_out[1])):
            blocks.append(box((wall_x1, seat_x), y, (-half_w, half_w)))
            blocks.append(box((wall_x1, self.lower_reach), y, (z_low, z_floor[0])))
        body = union([
            box((wall_x0, wall_x1), y_out, (z_low, half_w + 1.2)),
            box((wall_x1, self.lower_reach), y_out, z_floor),
            *blocks,
        ])

        cable = [
            # pitch servo lead (either end of the case)
            box((wall_x1, wall_x1 + 6.5), (y_out[0] - 1, y_out[1] + 1), (-2.6, 2.6)),
            # roll servo lead
            box((-2.6, 2.6), (y_out[0] - 1, y_out[1] + 1), (z_floor[0] - 6.5, z_floor[0])),
        ]
        return cut(body, [
            pitch.case_cutter(), roll.case_cutter(bottom_gap=c),
            pitch.pilot_cutter(depth=8.0, diameter=tol.m2_tap),
            roll.pilot_cutter(depth=8.0, diameter=tol.m2_tap),
            cyl_x((wall_x0 - 1, wall_x1 + 1), 0, 0, tol.m3_clear / 2),
            *cable,
        ])

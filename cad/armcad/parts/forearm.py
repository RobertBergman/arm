"""Forearm: carries the elbow servo and ends in the wrist fork."""
from __future__ import annotations

import cadquery as cq

from ..components import arm_horn_cutters
from ..geometry import Frame, box, cut, cyl_x, prism_yz, union
from .base_part import PrintedPart


class Forearm(PrintedPart):
    name = "forearm"
    link = "forearm"
    print_note = "Print as modelled, floor on the bed. No supports."

    tab_wall = 5.0
    floor = 2.5
    cross_wall = 4.0
    floor_stop = 37.0        # floor ends this far before the wrist axis

    def build(self) -> cq.Shape:
        lay, tol = self.layout, self.layout.tol
        s = lay.std
        length = lay.forearm_length
        servo = lay.elbow_servo()
        seat_x = lay.pitch_seat_x
        wall_x0, wall_x1 = lay.idler_wall_x
        half_w = s.width / 2 + tol.pocket
        z_bot, z_top = -half_w - self.floor - 0.5, half_w
        y_rear = s.shaft_to_short_end + s.tab_extension + 3.75
        y_front = -(s.shaft_to_long_end + s.tab_extension + 3.75)
        y_cross = y_front - self.cross_wall
        p_in, p_out = lay.wrist_half, lay.wrist_half + lay.prong
        z = (z_bot, z_top)

        nose = 8.0
        rail = [(y_front, z_top), (-length - 3, z_top), (-length - 3 - nose, z_top - nose),
                (-length - 3 - nose, z_bot + nose), (-length - 3, z_bot), (y_front, z_bot)]
        body = union([
            box((wall_x0, seat_x), (y_cross, y_rear), (z_bot, z_bot + self.floor)),
            box((seat_x - self.tab_wall, seat_x), (y_front, y_rear), z),
            box((wall_x0, wall_x1), (y_cross, y_rear), z),
            box((wall_x0, p_out), (y_cross, y_front), z),
            prism_yz(rail, (p_in, p_out)),
            prism_yz(rail, (-p_out, -p_in)),
            box((-p_out, p_out), (-length + self.floor_stop, y_cross),
                (z_bot, z_bot + self.floor)),
        ])

        horn_centre = Frame(origin=(p_in, -length, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
        ties = [box((-p_out - 1, -p_in + 1), (y, y + 5), (z_bot + 6, z_bot + 9))
                for y in (-58.0, -72.0)]
        return cut(body, [
            servo.case_cutter(open_towards="-y"),
            servo.pilot_cutter(depth=self.tab_wall + 1, diameter=tol.m3_tap),
            cyl_x((wall_x0 - 1, wall_x1 + 1), 0, 0, tol.m3_clear / 2),
            *arm_horn_cutters(horn_centre, lay.arm_horn, tol, lay.prong),
            cyl_x((-p_out - 1, -p_in + 1), -length, 0, tol.m3_tap / 2),
            *ties,
        ])

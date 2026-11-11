"""Turret: rides on the base servo horn and carries the shoulder servo."""
from __future__ import annotations

import cadquery as cq

from ..components import disc_horn_cutters
from ..geometry import Frame, box, cut, cyl_x, cyl_z, prism, union
from .base_part import PrintedPart


def _rib(x_wall: float, y: tuple, z0: float, height: float, reach: float) -> cq.Shape:
    """Triangular buttress in the XZ plane, extruded along Y."""
    y0, y1 = sorted(y)
    frame = Frame(origin=(0, y1, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))
    pts = [(x_wall, z0), (x_wall, z0 + height), (x_wall - reach, z0)]
    return prism(pts, frame, y1 - y0)


class Turret(PrintedPart):
    name = "turret"
    link = "turret"
    print_note = "Print as modelled, disc on the bed. No supports."

    disc_radius = 32.0
    tongue_length = 42.0
    tab_wall = 5.0
    horn_seat = 4.0           # plate thickness left under the horn screw heads
    head_pocket_radius = 12.5

    def build(self) -> cq.Shape:
        lay, tol = self.layout, self.layout.tol
        s = lay.std
        t = lay.turret_plate
        zs = lay.shoulder_above_turret
        servo = lay.shoulder_servo()
        seat_x = lay.pitch_seat_x
        wall_x0, wall_x1 = lay.idler_wall_x
        half = -wall_x0                      # the turret is as wide as the idler wall
        half_w = s.width / 2 + tol.pocket

        y_front = -(s.shaft_to_short_end + s.tab_extension + 3.75)
        y_rear = s.shaft_to_long_end + s.tab_extension + 3.75
        plate = union([cyl_z(0, 0, (0, t), self.disc_radius),
                       box((-half, half), (0, self.tongue_length), (0, t))])
        plate = plate.intersect(box((-half, half), (-100, 100), (0, t)))

        wall = box((seat_x - self.tab_wall, seat_x), (y_front, y_rear), (t, zs + half_w))
        idler = union([box((wall_x0, wall_x1), (-9, 9), (t, zs)),
                       cyl_x((wall_x0, wall_x1), 0, zs, 9.0)])
        ribs = [_rib(seat_x - self.tab_wall, (y_front, y_front + 4), t, 16, 14),
                _rib(seat_x - self.tab_wall, (y_rear - 4, y_rear), t, 16, 14)]
        body = union([plate, wall, idler, *ribs])

        horn = disc_horn_cutters(Frame(), lay.disc_horn, tol, self.horn_seat)
        head_pocket = cyl_z(0, 0, (self.horn_seat, zs - half_w + 0.01),
                            self.head_pocket_radius)
        return cut(body, [
            servo.case_cutter(open_towards="+y"),
            servo.pilot_cutter(depth=self.tab_wall + 1, diameter=tol.m3_tap),
            cyl_x((wall_x0 - 1, wall_x1 + 1), 0, zs, tol.m3_clear / 2),
            head_pocket, *horn,
        ])

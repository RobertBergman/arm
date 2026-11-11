"""Upper arm: U-channel whose side plates bolt to the shoulder and elbow horns."""
from __future__ import annotations

import cadquery as cq

from ..components import disc_horn_cutters
from ..geometry import Frame, box, cut, cyl_x, prism_yz, rotation, union
from .base_part import PrintedPart


class UpperArm(PrintedPart):
    name = "upper_arm"
    link = "upper_arm"
    print_note = "Print on its back (the closed side of the channel). No supports."
    print_rotation = rotation((1, 0, 0), -90)

    half_width = 18.0
    back_wall = (14.0, 18.0)       # y range of the web joining the two plates
    back_span = (28.0, 80.0)       # z range of that web

    def _plate(self, x: tuple) -> cq.Shape:
        length, r = self.layout.upper_arm_length, self.half_width
        chamfer = 10.5
        # squared-off rear corners (chamfered 45 deg) so the part sits flat on the bed
        rear = [(r, -r + chamfer), (r - chamfer, -r), (0, -r), (0, length + r),
                (r - chamfer, length + r), (r, length + r - chamfer)]
        return union([box(x, (-r, r), (0, length)),
                      cyl_x(x, 0, 0, r), cyl_x(x, 0, length, r),
                      prism_yz(rear, x)])

    def build(self) -> cq.Shape:
        lay, tol = self.layout, self.layout.tol
        inner, outer = lay.fork_half, lay.fork_half + lay.plate
        length = lay.upper_arm_length

        drive, idler = self._plate((inner, outer)), self._plate((-outer, -inner))
        web = box((-inner, inner), self.back_wall, self.back_span)
        body = union([drive, idler, web])

        tools = []
        for z in (0.0, length):
            centre = Frame(origin=(inner, 0, z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
            tools += disc_horn_cutters(centre, lay.disc_horn, tol, lay.plate)
            tools.append(cyl_x((-outer - 1, -inner + 1), 0, z, tol.m3_tap / 2))
        # lightening windows, pointed so they print without bridges
        window = [(8.0, 36.0), (8.0, 70.0), (-9.0, 53.0)]
        tools += [prism_yz(window, (inner - 1, outer + 1)),
                  prism_yz(window, (-outer - 1, -inner + 1))]
        # cable-tie slots through the web
        for z in (38.0, 66.0):
            for x in (-9.0, 6.0):
                tools.append(box((x, x + 3), (self.back_wall[0] - 1, self.back_wall[1] + 1),
                                 (z, z + 5)))
        return cut(body, tools)

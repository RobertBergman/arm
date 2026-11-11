"""Grid base plate (printed as six tiles), joining keys and electronics tray."""
from __future__ import annotations

from typing import List, Tuple

import cadquery as cq

from ..geometry import Frame, box, cut, cyl_z, prism, union
from ..specs import PCA9685_BOARD
from .base import Base
from .base_part import PrintedPart

TILE = 120.0
KEY_DEPTH = 4.0


def key_outline(grow: float = 0.0) -> List[Tuple[float, float]]:
    """Bow-tie key lying along X, centred on a tile seam."""
    neck, end, half = 3.0 + grow, 6.0 + grow, 12.0 + grow
    return [(-half, -end), (0, -neck), (half, -end), (half, end), (0, neck), (-half, end)]


def seam_keys() -> List[Tuple[float, float, bool]]:
    """(x, y, along_x) of every key: along_x keys bridge a seam at constant x."""
    keys = [(sx, y, True) for sx in (-60.0, 60.0) for y in (-90.0, -30.0, 30.0, 90.0)]
    keys += [(x, 0.0, False) for x in (-150.0, -90.0, -30.0, 30.0, 90.0, 150.0)]
    return keys


class Tray(PrintedPart):
    """Open tray for the servo driver and the microcontroller."""

    name = "electronics_tray"
    print_note = "Print flat. No supports."

    size = (96.0, 64.0)
    origin = (-173.0, 54.0)          # world position of the tray corner
    plate = 2.5
    standoff = 5.0

    def mount_holes(self) -> List[Tuple[float, float]]:
        (w, d), (ox, oy) = self.size, self.origin
        return [(ox + 5, oy + 5), (ox + w - 5, oy + 5),
                (ox + 5, oy + d - 5), (ox + w - 5, oy + d - 5)]

    def build(self) -> cq.Shape:
        tol = self.layout.tol
        (w, d), (ox, oy) = self.size, self.origin
        board = PCA9685_BOARD
        bx, by = ox + 16.0, oy + d - 8.0 - board.hole_dy   # first standoff
        posts = [(bx + i * board.hole_dx, by + j * board.hole_dy)
                 for i in (0, 1) for j in (0, 1)]
        z_top = self.plate + self.standoff
        body = union([box((ox, ox + w), (oy, oy + d), (0, self.plate)),
                      *[cyl_z(x, y, (0, z_top), 3.2) for x, y in posts]])
        tools = [cyl_z(x, y, (self.plate, z_top + 1), tol.m25_tap / 2) for x, y in posts]
        tools += [cyl_z(x, y, (-1, self.plate + 1), tol.m3_clear / 2)
                  for x, y in self.mount_holes()]
        # cable-tie slots that strap down the microcontroller board and the DC jack
        for x in (ox + 18, ox + 44, ox + 70):
            for y in (oy + 6, oy + 30):
                tools.append(box((x, x + 6), (y, y + 2.5), (-1, self.plate + 1)))
        return cut(body, tools)


class GridTile(PrintedPart):
    """One 120 x 120 mm tile of the grid plate."""

    print_note = "Print top face up. No supports."

    groove = (1.2, 0.7)

    def __init__(self, layout, column: int, row: int):
        super().__init__(layout)
        self.column, self.row = column, row
        self.name = f"grid_tile_c{column}_r{row}"

    def _plate(self) -> cq.Shape:
        lay = self.layout
        (w, d), t = lay.grid_size, lay.grid_thickness
        full = cq.Workplane("XY").rect(w, d).extrude(-t).edges("|Z").fillet(8.0).val()
        x0, y0 = -w / 2 + self.column * TILE, -d / 2 + self.row * TILE
        return full.intersect(box((x0, x0 + TILE), (y0, y0 + TILE), (-t - 1, 1)))

    def _engraving(self) -> List[cq.Shape]:
        lay = self.layout
        gw, gd = self.groove
        z = (-gd, 1)
        tools = [box((x - gw / 2, x + gw / 2), (-108, 108), z) for x in range(-150, 151, 30)]
        tools += [box((-168, 168), (y - gw / 2, y + gw / 2), z) for y in range(-90, 91, 30)]
        for cx, cy in (lay.zone_a, lay.zone_b):
            hw, hd = lay.zone_size[0] / 2, lay.zone_size[1] / 2
            outer = box((cx - hw - 0.8, cx + hw + 0.8), (cy - hd - 0.8, cy + hd + 0.8), z)
            inner = box((cx - hw + 0.8, cx + hw - 0.8), (cy - hd + 0.8, cy + hd - 0.8),
                        (-gd - 1, 2))
            tools.append(outer.cut(inner))
        return tools

    def _holes(self) -> List[cq.Shape]:
        lay, tol = self.layout, self.layout.tol
        t = lay.grid_thickness
        z = (-t - 1, 1)
        holes = [cyl_z(sx * 166, sy * 106, z, 2.25) for sx in (-1, 1) for sy in (-1, 1)]
        bx, by = lay.base_xy
        holes += [cyl_z(bx + x, by + y, z, tol.m3_tap / 2)
                  for x, y in Base(lay).bolt_positions()]
        holes += [cyl_z(x, y, z, tol.m3_tap / 2) for x, y in Tray(lay).mount_holes()]
        for x, y, along_x in seam_keys():
            frame = Frame(origin=(x, y, -t - 1),
                          x_dir=(1, 0, 0) if along_x else (0, 1, 0))
            holes.append(prism(key_outline(grow=0.2), frame, KEY_DEPTH + 1))
        return holes

    def build(self) -> cq.Shape:
        return cut(self._plate(), self._engraving() + self._holes())


class GridKey(PrintedPart):
    name = "grid_key"
    quantity = len(seam_keys())
    print_note = "Print flat. Press into the pockets under the tile seams."

    def build(self) -> cq.Shape:
        return prism(key_outline(), Frame(), KEY_DEPTH - 0.3)

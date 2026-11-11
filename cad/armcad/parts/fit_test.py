"""Small coupon to print first: checks your printer's fit on every critical feature."""
from __future__ import annotations

import cadquery as cq

from ..components import MountedServo
from ..geometry import Frame, box, cut, cyl_z
from .base_part import PrintedPart


class FitTest(PrintedPart):
    name = "fit_test_coupon"
    print_note = ("Print this first. Both servos must drop into their windows without "
                  "force and the screws must bite in the pilot holes.")

    thickness = 6.0

    def build(self) -> cq.Shape:
        lay, tol = self.layout, self.layout.tol
        t = self.thickness
        std = MountedServo(lay.std_servo, Frame())
        micro = MountedServo(lay.micro_servo, Frame(origin=(4.0, 26.0, 0.0)))
        body = box((-21, 41), (-14, 38), (-t, 0))
        return cut(body, [
            std.case_cutter(), std.pilot_cutter(t + 1, tol.m3_tap),
            micro.case_cutter(), micro.pilot_cutter(t + 1, tol.m2_tap),
            cyl_z(-15, 26, (-t - 1, 1), tol.m3_tap / 2),      # M3 screw should thread in
            cyl_z(35, 26, (-t - 1, 1), tol.m3_clear / 2),     # M3 screw should slide through
        ])

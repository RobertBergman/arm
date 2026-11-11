"""Solid models of the purchased components.

Each model serves two purposes: it is placed in the assembly so interference
checks see the real part, and it generates the cutters (pockets, pilot holes)
that the printed parts subtract, so a pocket can never drift out of sync with
the component it is meant to hold.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

import cadquery as cq

from .geometry import Frame, box, cyl_z, prism, slot_xy, union
from .specs import ArmHornSpec, DiscHornSpec, PrintTolerance, ServoSpec

_OPEN = 80.0  # length used to "open" a pocket towards one side


class TabServo:
    """Geometry of a tab-mounted hobby servo, expressed in the servo frame."""

    def __init__(self, spec: ServoSpec, tol: PrintTolerance):
        self.spec = spec
        self.tol = tol

    # -- the component itself -------------------------------------------------
    def solid(self) -> cq.Shape:
        s = self.spec
        half_w = s.width / 2.0
        case = box((-s.shaft_to_short_end, s.shaft_to_long_end), (-half_w, half_w),
                   (-s.depth_below_seat, s.case_above_seat))
        tabs = box((-s.shaft_to_short_end - s.tab_extension,
                    s.shaft_to_long_end + s.tab_extension),
                   (-half_w, half_w), (0.0, s.tab_thickness))
        boss = cyl_z(0, 0, (s.case_above_seat, s.boss_top), s.boss_radius)
        spline = cyl_z(0, 0, (s.boss_top, s.spline_top), s.spline_radius)
        return union([case, tabs, boss, spline])

    # -- cutters --------------------------------------------------------------
    def case_cutter(self, open_towards: Optional[str] = None,
                    bottom_gap: float = 0.5) -> cq.Shape:
        """Pocket for the part of the case below the tabs.

        ``open_towards`` ("+y" or "-y") turns the window into a U-slot so the
        servo can be dropped in and no bridge has to be printed.
        """
        s, c = self.spec, self.tol.pocket
        half_w = s.width / 2.0 + c
        y = [-half_w, half_w]
        if open_towards == "+y":
            y[1] = _OPEN
        elif open_towards == "-y":
            y[0] = -_OPEN
        elif open_towards is not None:
            raise ValueError(f"unsupported opening direction {open_towards!r}")
        return box((-s.shaft_to_short_end - c, s.shaft_to_long_end + c), y,
                   (-s.depth_below_seat - bottom_gap, 0.0))

    def upper_cutter(self, height: float) -> cq.Shape:
        """Clearance for tabs and upper case when the servo is sunk into a part."""
        s, c = self.spec, self.tol.pocket
        half_w = s.width / 2.0 + c
        ext = s.tab_extension + c + 0.2
        return box((-s.shaft_to_short_end - ext, s.shaft_to_long_end + ext),
                   (-half_w, half_w), (0.0, height))

    def pilot_cutter(self, depth: float, diameter: float) -> cq.Shape:
        """Screw pilot holes below the tab seats."""
        s = self.spec
        holes = [cyl_z(hx, hy, (-depth, 0.0), diameter / 2.0)
                 for hx in s.hole_x for hy in s.hole_y]
        return union(holes)


class DiscHorn:
    """Aluminium disc horn, modelled in the frame of the servo it sits on."""

    def __init__(self, spec: DiscHornSpec, servo: ServoSpec):
        self.spec = spec
        self.servo = servo

    def solid(self) -> cq.Shape:
        h, s = self.spec, self.servo
        flange = cyl_z(0, 0, (s.horn_face - h.flange_thickness, s.horn_face), h.radius)
        hub = cyl_z(0, 0, (s.boss_top + 0.2, s.horn_face - h.flange_thickness),
                    h.hub_radius)
        return union([flange, hub])


class ArmHorn:
    """Plastic double-arm horn, modelled in the frame of its servo."""

    def __init__(self, spec: ArmHornSpec, servo: ServoSpec):
        self.spec = spec
        self.servo = servo

    def solid(self, direction_deg: float = 0.0) -> cq.Shape:
        h, s = self.spec, self.servo
        top = s.horn_face
        hub = cyl_z(0, 0, (top - h.hub_height, top), h.hub_radius)
        a = math.radians(direction_deg)
        ux, uy = math.cos(a), math.sin(a)
        nx, ny = -uy, ux
        r, wr, wt = h.half_length, h.arm_width_root / 2.0, h.arm_width_tip / 2.0
        pts = [(nx * wr, ny * wr),
               (ux * r + nx * wt, uy * r + ny * wt),
               (ux * r - nx * wt, uy * r - ny * wt),
               (-nx * wr, -ny * wr),
               (-ux * r - nx * wt, -uy * r - ny * wt),
               (-ux * r + nx * wt, -uy * r + ny * wt)]
        arms = prism(pts, Frame(origin=(0, 0, top - h.arm_thickness)), h.arm_thickness)
        return union([hub, arms])


@dataclass(frozen=True)
class MountedServo:
    """A servo fixed at a known frame inside a printed part."""

    servo: TabServo
    frame: Frame

    def solid(self) -> cq.Shape:
        return self.frame.place(self.servo.solid())

    def case_cutter(self, open_towards: Optional[str] = None,
                    bottom_gap: float = 0.5) -> cq.Shape:
        return self.frame.place(self.servo.case_cutter(open_towards, bottom_gap))

    def upper_cutter(self, height: float) -> cq.Shape:
        return self.frame.place(self.servo.upper_cutter(height))

    def pilot_cutter(self, depth: float, diameter: float) -> cq.Shape:
        return self.frame.place(self.servo.pilot_cutter(depth, diameter))

    def place(self, shape_in_servo_frame: cq.Shape) -> cq.Shape:
        return self.frame.place(shape_in_servo_frame)


def disc_horn_cutters(center_frame: Frame, horn: DiscHornSpec, tol: PrintTolerance,
                      thickness: float, start_deg: float = 45.0) -> List[cq.Shape]:
    """Hole pattern that bolts a printed plate to a disc horn.

    ``center_frame`` sits on the shaft axis with +Z pointing through the plate.
    The bolt holes are radial slots so horns with a 13-16 mm bolt circle fit.
    """
    z = (-0.5, thickness + 0.5)
    r_nominal = horn.bolt_circle / 2.0
    tools = [cyl_z(0, 0, z, 4.25)]  # screwdriver access to the horn's centre screw
    for i in range(horn.bolt_count):
        angle = start_deg + i * 360.0 / horn.bolt_count
        tools.append(slot_xy((0, 0), angle, r_nominal - 0.75, r_nominal + 1.25,
                             tol.m3_clear, z))
    return [center_frame.place(t) for t in tools]


def arm_horn_cutters(center_frame: Frame, horn: ArmHornSpec, tol: PrintTolerance,
                     thickness: float, directions_deg=(0.0,)) -> List[cq.Shape]:
    """Access hole plus screw slots for fixing a plate to a plastic arm horn."""
    z = (-0.5, thickness + 0.5)
    tools = [cyl_z(0, 0, z, 2.6)]
    for angle in directions_deg:
        tools.append(slot_xy((0, 0), angle, horn.hole_r_min, horn.hole_r_max,
                             tol.m2_clear, z))
    return [center_frame.place(t) for t in tools]

"""Catalogue of every printed part."""
from __future__ import annotations

from typing import List

from ..layout import ArmLayout
from .base import Base, TurretShim
from .base_part import PrintedPart
from .fit_test import FitTest
from .forearm import Forearm
from .gripper import Finger, GripperBody
from .turret import Turret
from .upper_arm import UpperArm
from .workcell import GridKey, GridTile, Tray
from .wrist import Wrist

SHIM_THICKNESSES = (0.6, 1.0, 1.4, 1.8)


def arm_parts(layout: ArmLayout) -> List[PrintedPart]:
    """The moving arm, one instance per printed part."""
    return [Base(layout), Turret(layout), UpperArm(layout), Forearm(layout),
            Wrist(layout), GripperBody(layout), Finger(layout, +1), Finger(layout, -1)]


def workcell_parts(layout: ArmLayout) -> List[PrintedPart]:
    tiles = [GridTile(layout, c, r) for r in range(2) for c in range(3)]
    return [*tiles, GridKey(layout), Tray(layout)]


def all_parts(layout: ArmLayout) -> List[PrintedPart]:
    shims = [TurretShim(layout, t) for t in SHIM_THICKNESSES]
    return [FitTest(layout), *arm_parts(layout), *shims, *workcell_parts(layout)]


__all__ = ["PrintedPart", "arm_parts", "workcell_parts", "all_parts", "SHIM_THICKNESSES"]

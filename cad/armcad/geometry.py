"""Small geometry helpers shared by every part and component.

All dimensions are millimetres.  Shapes are plain ``cadquery.Shape`` objects so
that parts stay independent of any particular modelling workflow.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple

import cadquery as cq

Vec3 = Tuple[float, float, float]
Point2 = Tuple[float, float]

X_AXIS: Vec3 = (1.0, 0.0, 0.0)
Y_AXIS: Vec3 = (0.0, 1.0, 0.0)
Z_AXIS: Vec3 = (0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Frame:
    """Right-handed coordinate frame described in its parent's coordinates."""

    origin: Vec3 = (0.0, 0.0, 0.0)
    x_dir: Vec3 = X_AXIS
    z_dir: Vec3 = Z_AXIS

    @property
    def location(self) -> cq.Location:
        plane = cq.Plane(origin=self.origin, xDir=self.x_dir, normal=self.z_dir)
        return cq.Location(plane)

    def place(self, shape: cq.Shape) -> cq.Shape:
        """Move a shape modelled in this frame into the parent frame."""
        return shape.moved(self.location)

    def point(self, local: Vec3) -> Vec3:
        v = self.location.wrapped.Transformation()
        p = cq.Vector(*local).toPnt().Transformed(v)
        return (p.X(), p.Y(), p.Z())


def translation(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Location:
    return cq.Location(cq.Vector(x, y, z))


def rotation(axis: Vec3, degrees: float) -> cq.Location:
    return cq.Location(cq.Vector(0, 0, 0), cq.Vector(*axis), degrees)


def box(x: Sequence[float], y: Sequence[float], z: Sequence[float]) -> cq.Shape:
    """Axis-aligned box from (min, max) pairs."""
    (x0, x1), (y0, y1), (z0, z1) = sorted(x), sorted(y), sorted(z)
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def cylinder(base: Vec3, axis: Vec3, radius: float, height: float) -> cq.Shape:
    return cq.Solid.makeCylinder(radius, height, cq.Vector(*base), cq.Vector(*axis))


def cyl_x(x: Sequence[float], y: float, z: float, radius: float) -> cq.Shape:
    x0, x1 = sorted(x)
    return cylinder((x0, y, z), X_AXIS, radius, x1 - x0)


def cyl_z(x: float, y: float, z: Sequence[float], radius: float) -> cq.Shape:
    z0, z1 = sorted(z)
    return cylinder((x, y, z0), Z_AXIS, radius, z1 - z0)


def prism(points: Iterable[Point2], frame: Frame, height: float) -> cq.Shape:
    """Extrude a closed polygon drawn in ``frame``'s XY plane along its +Z."""
    wp = cq.Workplane("XY").polyline(list(points)).close().extrude(height)
    return frame.place(wp.val())


def prism_yz(points: Iterable[Point2], x: Sequence[float]) -> cq.Shape:
    """Extrude a polygon given as (y, z) points between two x positions."""
    x0, x1 = sorted(x)
    frame = Frame(origin=(x0, 0.0, 0.0), x_dir=Y_AXIS, z_dir=X_AXIS)
    return prism(points, frame, x1 - x0)


def union(shapes: Iterable[cq.Shape]) -> cq.Shape:
    shapes = list(shapes)
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.fuse(shape)
    return result.clean()


def cut(shape: cq.Shape, tools: Iterable[cq.Shape]) -> cq.Shape:
    for tool in tools:
        shape = shape.cut(tool)
    return shape.clean()


def slot_yz(center: Point2, direction_deg: float, r0: float, r1: float,
            width: float, x: Sequence[float]) -> cq.Shape:
    """Radial slot (stadium) in a YZ plate, running from r0 to r1 from ``center``."""
    a = math.radians(direction_deg)
    ux, uy = math.cos(a), math.sin(a)
    nx, ny = -uy, ux
    h = width / 2.0
    cy, cz = center
    pts = [
        (cy + ux * r0 + nx * h, cz + uy * r0 + ny * h),
        (cy + ux * r1 + nx * h, cz + uy * r1 + ny * h),
        (cy + ux * r1 - nx * h, cz + uy * r1 - ny * h),
        (cy + ux * r0 - nx * h, cz + uy * r0 - ny * h),
    ]
    body = prism_yz(pts, x)
    ends = [cyl_x(x, cy + ux * r, cz + uy * r, h) for r in (r0, r1)]
    return union([body, *ends])


def slot_xy(center: Point2, direction_deg: float, r0: float, r1: float,
            width: float, z: Sequence[float]) -> cq.Shape:
    """Radial slot in an XY plate."""
    a = math.radians(direction_deg)
    ux, uy = math.cos(a), math.sin(a)
    nx, ny = -uy, ux
    h = width / 2.0
    cx, cy = center
    pts = [
        (cx + ux * r0 + nx * h, cy + uy * r0 + ny * h),
        (cx + ux * r1 + nx * h, cy + uy * r1 + ny * h),
        (cx + ux * r1 - nx * h, cy + uy * r1 - ny * h),
        (cx + ux * r0 - nx * h, cy + uy * r0 - ny * h),
    ]
    z0, z1 = sorted(z)
    body = prism(pts, Frame(origin=(0, 0, z0)), z1 - z0)
    ends = [cyl_z(cx + ux * r, cy + uy * r, z, h) for r in (r0, r1)]
    return union([body, *ends])

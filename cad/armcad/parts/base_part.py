"""Common contract for every printed part."""
from __future__ import annotations

from abc import ABC, abstractmethod
from functools import cached_property

import cadquery as cq

from ..geometry import rotation
from ..layout import ArmLayout


class PrintedPart(ABC):
    """A part that is 3D printed.

    Subclasses only describe geometry.  Exporting, placing in the assembly and
    checking are handled elsewhere and depend solely on this interface.
    """

    name: str = ""
    link: str = "world"            # assembly link the part is rigidly attached to
    quantity: int = 1
    print_note: str = ""
    #: rotation that lays the part on the bed in its intended print orientation
    print_rotation: cq.Location = rotation((1, 0, 0), 0)

    def __init__(self, layout: ArmLayout):
        self.layout = layout

    @abstractmethod
    def build(self) -> cq.Shape:
        """Return the part in its link frame (see ``layout``)."""

    @cached_property
    def shape(self) -> cq.Shape:
        return self.build()

    def printable(self) -> cq.Shape:
        """The part oriented for printing, resting on z = 0."""
        oriented = self.shape.moved(self.print_rotation)
        bb = oriented.BoundingBox()
        return oriented.moved(cq.Location(cq.Vector(-bb.xmin, -bb.ymin, -bb.zmin)))

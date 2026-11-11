"""Writers for the files that leave this project: STL for printing, STEP for CAD."""
from __future__ import annotations

from pathlib import Path
from typing import Dict, Iterable

import cadquery as cq

from .parts import PrintedPart


class StlExporter:
    """Writes each part in its print orientation, resting on the bed."""

    def __init__(self, directory: Path, tolerance: float = 0.05, angular: float = 0.2):
        self.directory = directory
        self.tolerance = tolerance
        self.angular = angular

    def export(self, parts: Iterable[PrintedPart]) -> Dict[str, Path]:
        self.directory.mkdir(parents=True, exist_ok=True)
        written = {}
        for part in parts:
            suffix = f"_x{part.quantity}" if part.quantity > 1 else ""
            path = self.directory / f"{part.name}{suffix}.stl"
            part.printable().exportStl(str(path), self.tolerance, self.angular)
            written[part.name] = path
        return written


class StepExporter:
    """Writes a named assembly so the design can be opened in any CAD package."""

    def __init__(self, path: Path):
        self.path = path

    def export(self, shapes: Dict[str, cq.Shape]) -> Path:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        assembly = cq.Assembly(name=self.path.stem)
        for name, shape in shapes.items():
            assembly.add(shape, name=name)
        assembly.save(str(self.path))
        return self.path

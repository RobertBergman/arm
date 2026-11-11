"""Places printed parts and purchased components for a given joint state."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import cadquery as cq

from .components import ArmHorn, DiscHorn
from .geometry import X_AXIS, Z_AXIS, box, rotation, translation
from .kinematics import ArmKinematics, JointState
from .layout import ArmLayout
from .parts import PrintedPart, arm_parts, workcell_parts
from .parts.base import TurretShim
from .specs import PRINT_DENSITY_G_PER_MM3

#: order of links from the ground up; each joint moves every later link
LINK_CHAIN = ("base", "turret", "upper_arm", "forearm", "wrist", "gripper")
FINGER_LINKS = ("finger_a", "finger_b")


@dataclass(frozen=True)
class Item:
    """One rigid body of the assembly, in the frame of the link that carries it."""

    name: str
    link: str
    shape: cq.Shape
    printed: bool
    mass_g: float


class ArmAssembly:
    def __init__(self, layout: ArmLayout, include_workcell: bool = True,
                 shim_thickness: float = 1.4):
        self.layout = layout
        self.kinematics = ArmKinematics(layout)
        self.items: List[Item] = []
        parts: List[PrintedPart] = [*arm_parts(layout), TurretShim(layout, shim_thickness)]
        if include_workcell:
            parts += [p for p in workcell_parts(layout) if p.name != "grid_key"]
        for part in parts:
            self._add(part.name, part.link, part.shape, printed=True)
        self._add_components()

    # -- construction ---------------------------------------------------------
    def _add(self, name: str, link: str, shape: cq.Shape, printed: bool,
             mass_g: float = None) -> None:
        if mass_g is None:
            mass_g = shape.Volume() * PRINT_DENSITY_G_PER_MM3
        self.items.append(Item(name, link, shape, printed, mass_g))

    def _add_components(self) -> None:
        lay = self.layout
        disc = DiscHorn(lay.disc_horn, lay.std).solid()
        arm_horn = ArmHorn(lay.arm_horn, lay.micro)
        std_m, micro_m = lay.std.mass_g, lay.micro.mass_g
        disc_m, horn_m = lay.disc_horn.mass_g, lay.arm_horn.mass_g
        l1, l2 = lay.upper_arm_length, lay.forearm_length

        def add(name, link, shape, mass):
            self._add(name, link, shape, printed=False, mass_g=mass)

        add("servo_turret", "base", lay.base_servo().solid(), std_m)
        add("horn_turret", "turret",
            lay.base_servo().place(disc).moved(translation(z=-lay.turret_z)), disc_m)
        add("servo_shoulder", "turret", lay.shoulder_servo().solid(), std_m)
        add("horn_shoulder", "upper_arm", lay.shoulder_servo().place(disc).moved(
            translation(z=-lay.shoulder_above_turret)), disc_m)
        add("horn_elbow", "upper_arm",
            lay.elbow_servo().place(disc).moved(translation(z=l1)), disc_m)
        add("servo_elbow", "forearm", lay.elbow_servo().solid(), std_m)
        add("horn_wrist_pitch", "forearm", lay.pitch_servo().place(
            arm_horn.solid(0.0)).moved(translation(y=-l2)), horn_m)
        add("servo_wrist_pitch", "wrist", lay.pitch_servo().solid(), micro_m)
        add("servo_wrist_roll", "wrist", lay.roll_servo().solid(), micro_m)
        add("horn_wrist_roll", "gripper", lay.roll_servo().place(
            arm_horn.solid(0.0)).moved(translation(z=-lay.roll_face_z)), horn_m)
        add("servo_grip", "gripper", lay.grip_servo().solid(), micro_m)
        add("horn_grip", "finger_a", lay.grip_servo().place(arm_horn.solid(106.7)).moved(
            translation(y=-lay.finger_pivot_half, z=-lay.grip_pivot_z)), horn_m)

    # -- kinematics -----------------------------------------------------------
    def link_locations(self, q: JointState) -> Dict[str, cq.Location]:
        lay = self.layout
        base = translation(lay.base_xy[0], lay.base_xy[1], 0.0)
        turret = base * translation(z=lay.turret_z) * rotation(Z_AXIS, q.turret)
        upper = turret * translation(z=lay.shoulder_above_turret) * rotation(X_AXIS, q.shoulder)
        fore = upper * translation(z=lay.upper_arm_length) * rotation(X_AXIS, q.elbow)
        wrist = fore * translation(y=-lay.forearm_length) * rotation(X_AXIS, q.wrist_pitch)
        grip = wrist * translation(z=lay.roll_face_z) * rotation(Z_AXIS, q.wrist_roll)
        phi = self.kinematics.finger_angle(q.grip)
        half, pz = lay.finger_pivot_half, lay.grip_pivot_z
        return {
            "world": translation(), "base": base, "turret": turret, "upper_arm": upper,
            "forearm": fore, "wrist": wrist, "gripper": grip,
            "finger_a": grip * translation(y=half, z=pz) * rotation(X_AXIS, phi),
            "finger_b": grip * translation(y=-half, z=pz) * rotation(X_AXIS, -phi),
        }

    def placed(self, q: JointState) -> Dict[str, cq.Shape]:
        """Every item moved to world coordinates for joint state ``q``."""
        locs = self.link_locations(q)
        return {item.name: item.shape.moved(locs[item.link]) for item in self.items}

    def links_after(self, link: str) -> List[str]:
        """Links carried by the joint that drives ``link`` (including itself)."""
        if link in FINGER_LINKS:
            return [link]
        return list(LINK_CHAIN[LINK_CHAIN.index(link):]) + list(FINGER_LINKS)

    def payload_proxy(self, q: JointState, size=(84.0, 38.0, 35.0)) -> cq.Shape:
        """The truck as a box held between the pads (pad centre 14 mm above its wheels)."""
        lay = self.layout
        grip = self.link_locations(q)["gripper"]
        z_mid = lay.grip_pivot_z - lay.pad_centre_below_pivot
        z0 = z_mid - 14.0
        shape = box((-size[0] / 2, size[0] / 2), (-size[1] / 2, size[1] / 2),
                    (z0, z0 + size[2]))
        return shape.moved(grip)

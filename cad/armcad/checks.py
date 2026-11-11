"""Verification that the printed parts and purchased components work together."""
from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import combinations
from typing import Dict, Iterable, List, Sequence, Tuple

import cadquery as cq

from .assembly import ArmAssembly
from .kinematics import JOINT_LIMITS, JointState, TransferPlan
from .parts import PrintedPart

#: a servo's spline sits inside its own horn by design
INTENDED_OVERLAPS = {
    frozenset(p) for p in [
        ("servo_turret", "horn_turret"), ("servo_shoulder", "horn_shoulder"),
        ("servo_elbow", "horn_elbow"), ("servo_wrist_pitch", "horn_wrist_pitch"),
        ("servo_wrist_roll", "horn_wrist_roll"), ("servo_grip", "horn_grip"),
    ]
}

#: (horn, driven part): faces that must touch for the joint to be bolted up
REQUIRED_CONTACTS = [
    ("horn_turret", "turret"), ("horn_shoulder", "upper_arm"), ("horn_elbow", "upper_arm"),
    ("horn_wrist_pitch", "forearm"), ("horn_wrist_roll", "gripper_body"),
    ("horn_grip", "finger_drive"),
    ("servo_turret", "base"), ("servo_shoulder", "turret"), ("servo_elbow", "forearm"),
    ("servo_wrist_pitch", "wrist"), ("servo_wrist_roll", "wrist"),
    ("servo_grip", "gripper_body"),
]


@dataclass(frozen=True)
class Finding:
    check: str
    ok: bool
    detail: str


def _boxes_overlap(a: cq.BoundBox, b: cq.BoundBox) -> bool:
    return not (a.xmax < b.xmin or b.xmax < a.xmin or a.ymax < b.ymin
                or b.ymax < a.ymin or a.zmax < b.zmin or b.zmax < a.zmin)


class InterferenceChecker:
    """Boolean-intersects every pair of bodies in a pose."""

    def __init__(self, assembly: ArmAssembly, tolerance_mm3: float = 0.5):
        self.assembly = assembly
        self.tolerance = tolerance_mm3

    def collisions(self, q: JointState, extra: Dict[str, cq.Shape] = None
                   ) -> List[Tuple[str, str, float]]:
        placed = self.assembly.placed(q)
        placed.update(extra or {})
        boxes = {name: shape.BoundingBox() for name, shape in placed.items()}
        hits = []
        for a, b in combinations(placed, 2):
            if frozenset((a, b)) in INTENDED_OVERLAPS:
                continue
            if not _boxes_overlap(boxes[a], boxes[b]):
                continue
            volume = placed[a].intersect(placed[b]).Volume()
            if volume > self.tolerance:
                hits.append((a, b, volume))
        return hits


def check_interference(assembly: ArmAssembly, poses: Dict[str, JointState]) -> List[Finding]:
    """No two bodies may overlap in any pose.

    Poses that handle the truck ("transfer ...", "pick ...") also carry a box
    the size of the truck between the jaws, so the payload is checked against
    the arm, the base and the grid as well.
    """
    checker = InterferenceChecker(assembly)
    findings = []
    for label, q in poses.items():
        with_payload = label.startswith(("transfer", "pick"))
        extra = {"payload": assembly.payload_proxy(q)} if with_payload else None
        hits = checker.collisions(q, extra)
        detail = "clear" if not hits else "; ".join(
            f"{a} x {b} = {v:.1f} mm3" for a, b, v in hits)
        findings.append(Finding(f"interference @ {label}", not hits, detail))
    return findings


def check_contacts(assembly: ArmAssembly, max_gap: float = 0.05) -> List[Finding]:
    placed = assembly.placed(JointState())
    findings = []
    for a, b in REQUIRED_CONTACTS:
        gap = placed[a].distance(placed[b])
        findings.append(Finding(f"seated: {a} on {b}", gap <= max_gap, f"gap {gap:.3f} mm"))
    return findings


def check_bed_fit(parts: Iterable[PrintedPart], bed: Sequence[float] = (180.0, 180.0, 180.0)
                  ) -> List[Finding]:
    findings = []
    for part in parts:
        bb = part.printable().BoundingBox()
        dims = sorted((bb.xlen, bb.ylen), reverse=True)
        ok = dims[0] <= bed[0] and dims[1] <= bed[1] and bb.zlen <= bed[2]
        findings.append(Finding(f"bed fit: {part.name}", ok,
                                f"{bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm"))
    return findings


class TorqueAnalysis:
    """Static gravity torque on each pitch joint versus servo stall torque."""

    G = 9.81

    def __init__(self, assembly: ArmAssembly, payload_g: float = 60.0):
        self.assembly = assembly
        self.payload_g = payload_g

    def _joint_torque_kgcm(self, q: JointState, link: str) -> float:
        asm = self.assembly
        locs = asm.link_locations(q)
        carried = set(asm.links_after(link))
        origin = locs[link].toTuple()[0]
        tip = (locs[link] * cq.Location(cq.Vector(1, 0, 0))).toTuple()[0]
        axis = cq.Vector(*tip) - cq.Vector(*origin)
        moment = cq.Vector(0, 0, 0)
        masses = [(item.mass_g, cq.Shape.centerOfMass(item.shape.moved(locs[item.link])))
                  for item in asm.items if item.link in carried]
        pad = asm.payload_proxy(q)
        masses.append((self.payload_g, cq.Shape.centerOfMass(pad)))
        for mass, centre in masses:
            arm = centre - cq.Vector(*origin)
            moment = moment + arm.cross(cq.Vector(0, 0, -mass))
        return abs(moment.dot(axis)) / 1000.0 / 10.0   # g.mm -> kg.cm

    def worst_case(self, poses: Dict[str, JointState]) -> List[Finding]:
        lay = self.assembly.layout
        joints = {"shoulder": ("upper_arm", lay.std), "elbow": ("forearm", lay.std),
                  "wrist_pitch": ("wrist", lay.micro)}
        findings = []
        for joint, (link, servo) in joints.items():
            worst_label, worst = max(
                ((label, self._joint_torque_kgcm(q, link)) for label, q in poses.items()),
                key=lambda kv: kv[1])
            margin = servo.stall_torque_kgcm / worst if worst else math.inf
            findings.append(Finding(
                f"torque: {joint}", margin >= 2.0,
                f"{worst:.2f} kg.cm worst case ({worst_label}) vs {servo.stall_torque_kgcm:.1f} "
                f"kg.cm stall on {servo.name}: x{margin:.1f} margin"))
        return findings


def verification_poses(assembly: ArmAssembly, plan: TransferPlan = TransferPlan()
                       ) -> Dict[str, JointState]:
    """Poses the design must survive: the transfer plus every joint limit."""
    kin = assembly.kinematics
    poses: Dict[str, JointState] = {"zero": JointState()}
    for i, q in enumerate(plan.waypoints(kin, steps=12)):
        poses[f"transfer step {i * 5:02d}/60"] = q
    pick = plan.waypoints(kin, steps=12)[0]
    poses["pick, jaws open"] = pick.with_(grip=plan.open_width)
    home = JointState()
    limit = {name: dict(zip(("min", "max"), span)) for name, span in JOINT_LIMITS.items()}
    # Each joint at both ends of its travel, in a posture where that is meaningful.
    for joint in ("turret", "wrist_roll", "grip"):
        for end, value in limit[joint].items():
            poses[f"{joint} {end}"] = home.with_(**{joint: value})
    poses["shoulder min"] = home.with_(shoulder=limit["shoulder"]["min"])
    poses["shoulder max (arm straight out)"] = JointState(
        shoulder=limit["shoulder"]["max"], elbow=-90.0, wrist_pitch=-90.0)
    poses["elbow min (arm straight up)"] = home.with_(elbow=limit["elbow"]["min"])
    poses["elbow max"] = JointState(shoulder=limit["shoulder"]["min"],
                                    elbow=limit["elbow"]["max"], wrist_pitch=-90.0)
    poses["wrist_pitch min (tool forward)"] = home.with_(wrist_pitch=limit["wrist_pitch"]["min"])
    poses["wrist_pitch max (tool back)"] = home.with_(wrist_pitch=limit["wrist_pitch"]["max"])
    poses["reach forward low"] = kin.solve(0.0, -110.0, 14.0)
    poses["reach near base"] = kin.solve(0.0, -30.0, 30.0)
    return poses

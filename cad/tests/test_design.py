"""Regression tests for the printable arm.  Run with:  pytest cad/tests"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from armcad.assembly import ArmAssembly  # noqa: E402
from armcad.checks import (TorqueAnalysis, check_bed_fit, check_contacts,  # noqa: E402
                           check_interference, verification_poses)
from armcad.kinematics import (JOINT_LIMITS, ArmKinematics, TransferPlan,  # noqa: E402
                               UnreachableError)
from armcad.layout import DEFAULT_LAYOUT  # noqa: E402
from armcad.parts import all_parts  # noqa: E402


@pytest.fixture(scope="module")
def assembly():
    return ArmAssembly(DEFAULT_LAYOUT)


@pytest.fixture(scope="module")
def kinematics():
    return ArmKinematics(DEFAULT_LAYOUT)


def test_every_part_is_one_valid_solid():
    for part in all_parts(DEFAULT_LAYOUT):
        assert part.shape.isValid(), part.name
        assert len(part.shape.Solids()) == 1, f"{part.name} is in several pieces"


def test_every_part_rests_on_the_bed_and_fits_a_180mm_printer():
    parts = all_parts(DEFAULT_LAYOUT)
    for part in parts:
        assert part.printable().BoundingBox().zmin == pytest.approx(0.0, abs=1e-6)
    assert all(f.ok for f in check_bed_fit(parts))


def test_stl_meshes_are_watertight():
    for part in all_parts(DEFAULT_LAYOUT):
        vertices, triangles = part.printable().tessellate(0.05, 0.2)
        # faces are meshed separately: weld coincident vertices before pairing edges
        ids, weld = {}, []
        for v in vertices:
            weld.append(ids.setdefault((round(v.x, 4), round(v.y, 4), round(v.z, 4)), len(ids)))
        edges = {}
        for a, b, c in ((weld[i], weld[j], weld[k]) for i, j, k in triangles):
            for edge in ((a, b), (b, c), (c, a)):
                edges[edge] = edges.get(edge, 0) + 1
        open_edges = [e for e in edges if edges.get((e[1], e[0]), 0) != edges[e]]
        assert not open_edges, f"{part.name}: {len(open_edges)} open mesh edges"


@pytest.mark.parametrize("target", [(-110, -20, 14), (110, -20, 14), (0, -74, 69),
                                    (60, -60, 30), (0, -110, 14)])
def test_inverse_kinematics_round_trip(kinematics, target):
    reached = kinematics.pad_centre(kinematics.solve(*target))
    assert reached == pytest.approx(target, abs=1e-6)


def test_out_of_reach_is_reported(kinematics):
    with pytest.raises(UnreachableError):
        kinematics.solve(400, 0, 0)


def test_transfer_stays_within_joint_limits_and_ends_on_zone_b(kinematics):
    path = TransferPlan().waypoints(kinematics)
    for q in path:
        for joint, (lo, hi) in JOINT_LIMITS.items():
            assert lo <= getattr(q, joint) <= hi
        assert q.wrist_pitch == pytest.approx(-(q.shoulder + q.elbow))   # tool vertical
        assert q.wrist_roll == pytest.approx(-q.turret)                  # heading held
    x, y, z = kinematics.pad_centre(path[-1])
    assert (x, y) == pytest.approx(DEFAULT_LAYOUT.zone_b)
    assert math.isclose(z, TransferPlan().grasp_height)


def test_servos_and_horns_are_seated(assembly):
    assert all(f.ok for f in check_contacts(assembly))


def test_no_interference_and_enough_torque(assembly):
    poses = verification_poses(assembly)
    failures = [f for f in check_interference(assembly, poses) if not f.ok]
    assert not failures, failures
    assert all(f.ok for f in TorqueAnalysis(assembly).worst_case(poses))

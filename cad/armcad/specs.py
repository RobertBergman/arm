"""Dimensions of the purchased components and the print tolerances.

This is the single place to edit if the parts you receive measure differently
from the nominal values below (clone servos vary by a few tenths of a mm).
Re-run ``python cad/build.py`` afterwards: every pocket, hole pattern and
joint spacing is derived from these numbers.

Servo frame convention (used by every servo, whatever its size):
    origin  on the output-shaft axis, in the plane of the mounting-tab
            underside (the face that seats on the printed part)
    +Z      along the shaft, towards the horn
    +X      along the case length, towards the long end of the case
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class PrintTolerance:
    """Clearances for a 0.4 mm nozzle FDM printer."""

    pocket: float = 0.3          # per-side clearance around a servo case
    running: float = 0.3         # gap between parts that move past each other
    m3_clear: float = 3.4        # M3 clearance hole diameter
    m3_tap: float = 2.8          # M3 screw threads directly into this hole
    m2_clear: float = 2.3        # M2 clearance hole / slot width
    m2_tap: float = 1.6          # M2 self-tapping pilot
    m25_tap: float = 2.1         # M2.5 self-tapping pilot


@dataclass(frozen=True)
class ServoSpec:
    """A hobby servo with two mounting tabs."""

    name: str
    shaft_to_short_end: float    # case end nearest the shaft  (-X extent)
    shaft_to_long_end: float     # case end farthest from shaft (+X extent)
    width: float                 # case width (Y)
    depth_below_seat: float      # tab underside -> case bottom
    case_above_seat: float       # tab underside -> top of the main case
    tab_extension: float         # how far each tab sticks out past the case
    tab_thickness: float
    hole_x: Tuple[float, ...]    # mounting hole positions along X
    hole_y: Tuple[float, ...]    # mounting hole positions along Y
    boss_radius: float           # round boss around the output shaft
    boss_top: float              # tab underside -> top of that boss
    spline_radius: float
    spline_top: float            # tab underside -> end of the spline
    horn_face: float             # tab underside -> outer face of fitted horn
    mass_g: float
    stall_torque_kgcm: float     # at 5 V


@dataclass(frozen=True)
class DiscHornSpec:
    """25T aluminium disc horn for the standard servos."""

    radius: float = 10.0
    flange_thickness: float = 2.5
    hub_radius: float = 4.5
    bolt_circle: float = 14.0    # 4 x M3 threaded holes
    bolt_count: int = 4
    mass_g: float = 2.0


@dataclass(frozen=True)
class ArmHornSpec:
    """Plastic double-arm horn shipped with every MG90S."""

    half_length: float = 16.5
    hub_radius: float = 3.6
    hub_height: float = 4.5
    arm_thickness: float = 1.6
    arm_width_root: float = 5.6
    arm_width_tip: float = 4.0
    hole_r_min: float = 6.5      # nearest usable screw hole from the centre
    hole_r_max: float = 15.5     # farthest usable screw hole from the centre
    mass_g: float = 0.6


@dataclass(frozen=True)
class BoardSpec:
    name: str
    length: float
    width: float
    hole_dx: float
    hole_dy: float


# DS3218MG-class "standard" digital servo, 20 kg.cm, 25T spline.
# MG996R / DS3225 share this footprint within the pocket clearance.
STANDARD_SERVO = ServoSpec(
    name="DS3218MG (270 deg)",
    shaft_to_short_end=10.0,
    shaft_to_long_end=30.0,
    width=20.0,
    depth_below_seat=28.0,
    case_above_seat=10.0,
    tab_extension=7.25,
    tab_thickness=2.5,
    hole_x=(-14.75, 34.75),
    hole_y=(-5.0, 5.0),
    boss_radius=6.5,
    boss_top=12.5,
    spline_radius=3.0,
    spline_top=16.5,
    horn_face=18.5,
    mass_g=60.0,
    stall_torque_kgcm=18.0,
)

# MG90S metal-gear micro servo.
MICRO_SERVO = ServoSpec(
    name="MG90S (180 deg)",
    shaft_to_short_end=6.2,
    shaft_to_long_end=16.6,
    width=12.2,
    depth_below_seat=15.9,
    case_above_seat=6.8,
    tab_extension=4.75,
    tab_thickness=2.5,
    hole_x=(-8.7, 19.1),
    hole_y=(0.0,),
    boss_radius=5.9,
    boss_top=10.8,
    spline_radius=2.4,
    spline_top=14.0,
    horn_face=16.6,
    mass_g=13.5,
    stall_torque_kgcm=1.8,
)

DISC_HORN = DiscHornSpec()
ARM_HORN = ArmHornSpec()
PCA9685_BOARD = BoardSpec("PCA9685 16-channel servo driver", 62.5, 25.4, 55.9, 19.05)
TOLERANCE = PrintTolerance()

# Density used for mass/torque estimates.  Deliberately conservative: it
# treats every printed part as solid PLA (real parts with infill are lighter).
PRINT_DENSITY_G_PER_MM3 = 1.24e-3

"""Derived layout of the arm: every joint position and servo frame.

The numbers here are *computed* from the component specs, so changing a servo
dimension in ``specs.py`` moves the mating parts with it.  Each link is
modelled in its own frame: origin at the joint that drives it, axes parallel
to the world axes when the arm is in its zero pose (upper arm vertical,
forearm horizontal pointing to -Y, tool pointing down).
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .components import MountedServo, TabServo
from .geometry import Frame, Vec3
from .specs import (ARM_HORN, DISC_HORN, MICRO_SERVO, STANDARD_SERVO, TOLERANCE,
                    ArmHornSpec, DiscHornSpec, PrintTolerance, ServoSpec)


@dataclass(frozen=True)
class ArmLayout:
    std: ServoSpec = STANDARD_SERVO
    micro: ServoSpec = MICRO_SERVO
    disc_horn: DiscHornSpec = DISC_HORN
    arm_horn: ArmHornSpec = ARM_HORN
    tol: PrintTolerance = TOLERANCE

    # -- workcell (matches the original concept model) ------------------------
    base_xy: tuple = (0.0, 65.0)
    grid_size: tuple = (360.0, 240.0)
    grid_thickness: float = 8.0
    zone_a: tuple = (-110.0, -20.0)
    zone_b: tuple = (110.0, -20.0)
    zone_size: tuple = (96.0, 48.0)

    # -- link lengths ---------------------------------------------------------
    upper_arm_length: float = 105.0
    forearm_length: float = 105.0

    # -- design choices -------------------------------------------------------
    base_seat_z: float = 32.0        # turret-servo tab seat above the grid
    shim_gap: float = 1.5            # nominal gap taken up by shims/washers
    turret_plate: float = 6.0
    shoulder_above_turret: float = 21.0
    plate: float = 4.0               # upper-arm side plate thickness
    idler_wall: float = 5.0
    servo_bottom_gap: float = 0.5
    wrist_wall: float = 3.0
    wrist_gap: float = 0.6
    wrist_mid_floor: float = 2.0
    prong: float = 4.0
    finger_pivot_half: float = 13.5  # half the gear centre distance
    finger_plate: float = 5.0
    grip_seat_x: float = -4.0
    grip_pivot_z: float = -10.9
    pad_centre_below_pivot: float = 50.0
    pad_thickness: float = 2.0       # stick-on rubber pad
    grip_nominal_width: float = 38.0  # jaw opening with the fingers parallel

    std_servo: TabServo = field(init=False)
    micro_servo: TabServo = field(init=False)

    def __post_init__(self):
        object.__setattr__(self, "std_servo", TabServo(self.std, self.tol))
        object.__setattr__(self, "micro_servo", TabServo(self.micro, self.tol))

    # -- big pitch joints (shoulder and elbow share one geometry) -------------
    @property
    def fork_half(self) -> float:
        """Half the inside spacing of the upper-arm side plates."""
        s = self.std
        span = (s.horn_face + s.depth_below_seat + self.servo_bottom_gap
                + self.idler_wall + self.shim_gap)
        return span / 2.0

    @property
    def pitch_seat_x(self) -> float:
        return self.fork_half - self.std.horn_face

    @property
    def idler_wall_x(self) -> tuple:
        inner = self.pitch_seat_x - self.std.depth_below_seat - self.servo_bottom_gap
        return (inner - self.idler_wall, inner)

    # -- vertical stack -------------------------------------------------------
    @property
    def turret_z(self) -> float:
        """World height of the turret underside (= base horn face)."""
        return self.base_seat_z + self.std.horn_face

    @property
    def base_top_z(self) -> float:
        return self.turret_z - self.shim_gap

    @property
    def shoulder_z(self) -> float:
        return self.turret_z + self.shoulder_above_turret

    # -- wrist ----------------------------------------------------------------
    @property
    def wrist_half(self) -> float:
        """Half the inside spacing of the forearm prongs."""
        m = self.micro
        span = (m.horn_face + m.depth_below_seat + self.servo_bottom_gap
                + self.wrist_wall + self.wrist_gap)
        return span / 2.0

    @property
    def wrist_seat_x(self) -> float:
        return self.wrist_half - self.micro.horn_face

    @property
    def wrist_wall_x(self) -> tuple:
        inner = self.wrist_seat_x - self.micro.depth_below_seat - self.servo_bottom_gap
        return (inner - self.wrist_wall, inner)

    @property
    def roll_seat_z(self) -> float:
        m = self.micro
        return -(m.width / 2.0 + self.tol.pocket + self.wrist_mid_floor
                 + self.tol.pocket + m.depth_below_seat)

    @property
    def roll_face_z(self) -> float:
        """Roll-horn outer face below the wrist pitch axis (gripper origin)."""
        return self.roll_seat_z - self.micro.horn_face

    # -- gripper --------------------------------------------------------------
    @property
    def finger_plane_x(self) -> float:
        return self.grip_seat_x + self.micro.horn_face

    @property
    def tool_length(self) -> float:
        """Wrist pitch axis to the centre of the grip pads."""
        return -self.roll_face_z - self.grip_pivot_z + self.pad_centre_below_pivot

    # -- servo mounting frames, each in its host link's frame -----------------
    def base_servo(self) -> MountedServo:
        return MountedServo(self.std_servo, Frame((0, 0, self.base_seat_z),
                                                  x_dir=(0, 1, 0), z_dir=(0, 0, 1)))

    def shoulder_servo(self) -> MountedServo:
        origin: Vec3 = (self.pitch_seat_x, 0, self.shoulder_above_turret)
        return MountedServo(self.std_servo, Frame(origin, x_dir=(0, 1, 0), z_dir=(1, 0, 0)))

    def elbow_servo(self) -> MountedServo:
        return MountedServo(self.std_servo, Frame((self.pitch_seat_x, 0, 0),
                                                  x_dir=(0, -1, 0), z_dir=(1, 0, 0)))

    def pitch_servo(self) -> MountedServo:
        return MountedServo(self.micro_servo, Frame((self.wrist_seat_x, 0, 0),
                                                    x_dir=(0, 1, 0), z_dir=(1, 0, 0)))

    def roll_servo(self) -> MountedServo:
        return MountedServo(self.micro_servo, Frame((0, 0, self.roll_seat_z),
                                                    x_dir=(0, 1, 0), z_dir=(0, 0, -1)))

    def grip_servo(self) -> MountedServo:
        origin: Vec3 = (self.grip_seat_x, self.finger_pivot_half, self.grip_pivot_z)
        return MountedServo(self.micro_servo, Frame(origin, x_dir=(0, -1, 0), z_dir=(1, 0, 0)))


DEFAULT_LAYOUT = ArmLayout()

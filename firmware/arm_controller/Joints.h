// Joint identifiers and the joint-space pose used throughout the firmware.
#pragma once

enum Joint : unsigned char {
  JOINT_TURRET = 0,
  JOINT_SHOULDER,
  JOINT_ELBOW,
  JOINT_WRIST_PITCH,
  JOINT_WRIST_ROLL,
  JOINT_GRIP,
  JOINT_COUNT
};

// Angles in degrees.  Zero pose: upper arm vertical, forearm horizontal,
// tool pointing down, fingers parallel.  The grip entry is the finger angle.
struct Pose {
  float q[JOINT_COUNT];
};

struct JointLimit {
  float minDeg;
  float maxDeg;
};

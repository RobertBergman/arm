// Closed-form kinematics for a vertical tool.  Pure C++: no Arduino
// dependencies, so it is unit-tested on a PC (see firmware/test).
#pragma once

#include "ArmGeometry.h"
#include "Joints.h"

class Kinematics {
 public:
  explicit Kinematics(const ArmGeometry& geometry) : g_(geometry) {}

  // Joint angles that put the pad centre at (x, y, z) on the grid with the
  // tool vertical.  heading is the payload yaw (0 = jaws close along Y).
  // Returns false (pose untouched) when the point is out of reach.
  bool solve(float x, float y, float z, float headingDeg, float gripWidth, Pose& pose) const;

  // Finger angle (deg from parallel) for a pad-to-pad opening in mm.
  float fingerAngle(float gripWidth) const;

 private:
  ArmGeometry g_;
};

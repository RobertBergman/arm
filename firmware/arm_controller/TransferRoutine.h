// The pick-and-place cycle: zone A -> zone B along a lift / slew / lower path.
#pragma once

#include "ArmGeometry.h"
#include "JointController.h"
#include "Kinematics.h"

struct TransferConfig {
  float pickX, pickY;    // zone A centre
  float placeX, placeY;  // zone B centre
  float graspZ;          // pad centre above the grid when gripping
  float lift;            // straight lift before/after the slew
  float apex;            // extra height at the middle of the slew
  float openWidth;       // jaw opening while approaching (mm)
  float holdWidth;       // jaw opening while carrying (slightly under the payload width)
};

class TransferRoutine {
 public:
  // Called after each new target; must run controller.update() until it settles.
  typedef void (*MoveFn)(JointController& controller);

  TransferRoutine(const Kinematics& kinematics, const ArmGeometry& geometry,
                  JointController& controller, const TransferConfig& config,
                  MoveFn runUntilSettled);

  // Runs one full cycle.  Returns false if a waypoint is out of reach.
  bool run();

  // Pose on the carry path, u = 0 (over the pick point) .. 1 (over the place point).
  bool carryPose(float u, float gripWidth, Pose& pose) const;

 private:
  bool moveTo(float x, float y, float z, float gripWidth);

  const Kinematics& kinematics_;
  ArmGeometry geometry_;
  JointController& controller_;
  TransferConfig config_;
  MoveFn runUntilSettled_;
};

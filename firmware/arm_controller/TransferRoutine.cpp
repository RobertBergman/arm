#include "TransferRoutine.h"

#include <math.h>

namespace {
const float kPi = 3.14159265f;
const int kSlewSteps = 36;

float ease(float u) { return 0.5f - 0.5f * cosf(kPi * u); }
}  // namespace

TransferRoutine::TransferRoutine(const Kinematics& kinematics, const ArmGeometry& geometry,
                                 JointController& controller, const TransferConfig& config,
                                 MoveFn runUntilSettled)
    : kinematics_(kinematics),
      geometry_(geometry),
      controller_(controller),
      config_(config),
      runUntilSettled_(runUntilSettled) {}

bool TransferRoutine::moveTo(float x, float y, float z, float gripWidth) {
  Pose pose;
  if (!kinematics_.solve(x, y, z, 0.0f, gripWidth, pose)) return false;
  controller_.setTarget(pose);
  runUntilSettled_(controller_);
  return true;
}

bool TransferRoutine::carryPose(float u, float gripWidth, Pose& pose) const {
  // Swing about the turret axis at constant radius so the payload never
  // passes through the arm's own base.
  const float ax = config_.pickX - geometry_.baseX, ay = config_.pickY - geometry_.baseY;
  const float bx = config_.placeX - geometry_.baseX, by = config_.placeY - geometry_.baseY;
  const float angA = atan2f(ax, -ay), angB = atan2f(bx, -by);
  const float radA = sqrtf(ax * ax + ay * ay), radB = sqrtf(bx * bx + by * by);
  const float e = ease(u);
  const float ang = angA + (angB - angA) * e;
  const float rad = radA + (radB - radA) * e;
  const float z = config_.graspZ + config_.lift + config_.apex * sinf(kPi * u);
  return kinematics_.solve(geometry_.baseX + rad * sinf(ang),
                           geometry_.baseY - rad * cosf(ang), z, 0.0f, gripWidth, pose);
}

bool TransferRoutine::run() {
  const TransferConfig& c = config_;
  const float high = c.graspZ + c.lift;

  // approach and grasp
  if (!moveTo(c.pickX, c.pickY, high, c.openWidth)) return false;
  if (!moveTo(c.pickX, c.pickY, c.graspZ, c.openWidth)) return false;
  if (!moveTo(c.pickX, c.pickY, c.graspZ, c.holdWidth)) return false;
  // straight lift, slew, straight lower
  if (!moveTo(c.pickX, c.pickY, high, c.holdWidth)) return false;
  for (int i = 1; i <= kSlewSteps; ++i) {
    Pose pose;
    if (!carryPose(static_cast<float>(i) / kSlewSteps, c.holdWidth, pose)) return false;
    controller_.setTarget(pose);
    runUntilSettled_(controller_);
  }
  if (!moveTo(c.placeX, c.placeY, c.graspZ, c.holdWidth)) return false;
  // release and retreat
  if (!moveTo(c.placeX, c.placeY, c.graspZ, c.openWidth)) return false;
  return moveTo(c.placeX, c.placeY, high, c.openWidth);
}

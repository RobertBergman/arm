#include "JointController.h"

namespace {
const int kMinPulseUs = 500;
const int kMaxPulseUs = 2500;

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

JointController::JointController(ServoOutput& output, ServoCalibration* calibration)
    : output_(output), calibration_(calibration) {
  for (int j = 0; j < JOINT_COUNT; ++j) current_.q[j] = target_.q[j] = 0.0f;
}

void JointController::begin(const Pose& startPose) {
  output_.begin();
  current_ = target_ = startPose;
  for (int j = 0; j < JOINT_COUNT; ++j) write(static_cast<Joint>(j));
}

void JointController::setTarget(Joint joint, float degrees) {
  const JointLimit& limit = calibration_[joint].limit;
  target_.q[joint] = clampf(degrees, limit.minDeg, limit.maxDeg);
}

void JointController::setTarget(const Pose& pose) {
  for (int j = 0; j < JOINT_COUNT; ++j) setTarget(static_cast<Joint>(j), pose.q[j]);
}

bool JointController::update(float dtSeconds) {
  bool moving = false;
  for (int j = 0; j < JOINT_COUNT; ++j) {
    const float error = target_.q[j] - current_.q[j];
    if (error == 0.0f) continue;
    const float step = calibration_[j].maxSpeed * dtSeconds;
    if (error > step) {
      current_.q[j] += step;
      moving = true;
    } else if (error < -step) {
      current_.q[j] -= step;
      moving = true;
    } else {
      current_.q[j] = target_.q[j];
    }
    write(static_cast<Joint>(j));
  }
  return moving;
}

void JointController::trim(Joint joint, int deltaUs) {
  calibration_[joint].centerUs += deltaUs;
  write(joint);
}

void JointController::releaseAll() {
  for (int j = 0; j < JOINT_COUNT; ++j) output_.release(calibration_[j].channel);
}

int JointController::pulseFor(Joint joint, float degrees) const {
  const ServoCalibration& c = calibration_[joint];
  const float pulse = c.centerUs + c.usPerDegree * degrees;
  return static_cast<int>(clampf(pulse, kMinPulseUs, kMaxPulseUs) + 0.5f);
}

void JointController::write(Joint joint) {
  output_.writeMicroseconds(calibration_[joint].channel, pulseFor(joint, current_.q[joint]));
}

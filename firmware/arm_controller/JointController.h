// Turns joint angles into servo pulses: calibration, limits and smooth motion.
#pragma once

#include "Joints.h"
#include "ServoOutput.h"

// How one joint's servo is wired and mounted.
struct ServoCalibration {
  unsigned char channel;  // PCA9685 output
  int centerUs;           // pulse width at the joint's zero pose (trim this)
  float usPerDegree;      // signed: negative when the servo turns the other way
  JointLimit limit;       // allowed joint angle
  float maxSpeed;         // deg/s when moving
};

class JointController {
 public:
  JointController(ServoOutput& output, ServoCalibration* calibration);

  void begin(const Pose& startPose);

  // Clamp to the joint limits and make it the new goal.
  void setTarget(const Pose& pose);
  void setTarget(Joint joint, float degrees);

  // Advance the motion by dtSeconds; returns true while still moving.
  bool update(float dtSeconds);

  // Move a joint's zero point by deltaUs and re-send its pulse.
  void trim(Joint joint, int deltaUs);

  void releaseAll();
  const Pose& current() const { return current_; }
  int pulseFor(Joint joint, float degrees) const;

 private:
  void write(Joint joint);

  ServoOutput& output_;
  ServoCalibration* calibration_;
  Pose current_;
  Pose target_;
};

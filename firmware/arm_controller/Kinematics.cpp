#include "Kinematics.h"

#include <math.h>

namespace {
const float kRadToDeg = 57.29577951f;
}

float Kinematics::fingerAngle(float gripWidth) const {
  float s = (gripWidth - g_.gripNominal) / (2.0f * g_.padRadius);
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  return asinf(s) * kRadToDeg;
}

bool Kinematics::solve(float x, float y, float z, float headingDeg, float gripWidth,
                       Pose& pose) const {
  const float dx = x - g_.baseX;
  const float dy = y - g_.baseY;
  const float reach = sqrtf(dx * dx + dy * dy);
  const float height = z + g_.toolLength - g_.shoulderHeight;
  const float l1 = g_.upperArm, l2 = g_.forearm;
  const float dist = sqrtf(reach * reach + height * height);
  if (dist >= l1 + l2 || dist <= fabsf(l1 - l2) || dist < 1.0f) return false;

  const float elevation = atan2f(height, reach);
  const float atShoulder = acosf((l1 * l1 + dist * dist - l2 * l2) / (2.0f * l1 * dist));
  const float atElbow = acosf((l1 * l1 + l2 * l2 - dist * dist) / (2.0f * l1 * l2));
  const float turret = atan2f(dx, -dy) * kRadToDeg;
  const float shoulder = 90.0f - (elevation + atShoulder) * kRadToDeg;
  const float forearmDrop = (3.14159265f - atElbow - elevation - atShoulder) * kRadToDeg;

  pose.q[JOINT_TURRET] = turret;
  pose.q[JOINT_SHOULDER] = shoulder;
  pose.q[JOINT_ELBOW] = forearmDrop - shoulder;
  pose.q[JOINT_WRIST_PITCH] = -forearmDrop;        // keeps the tool vertical
  pose.q[JOINT_WRIST_ROLL] = headingDeg - turret;  // keeps the payload heading
  pose.q[JOINT_GRIP] = fingerAngle(gripWidth);
  return true;
}

// Per-arm settings.  The values to edit after assembly are in ArmConfig.cpp.
#pragma once

#include "JointController.h"
#include "TransferRoutine.h"

// Servo wiring, direction, zero trim, limits and speed for every joint.
extern ServoCalibration ARM_CALIBRATION[JOINT_COUNT];

// Pick-and-place task on the grid (millimetres, grid coordinates).
// holdWidth is set for a 38 mm wide truck; use (width - 1.5) for other payloads.
//                               pick (zone A)    place (zone B)  graspZ lift  apex  open   hold
const TransferConfig TRANSFER = {-110.0f, -20.0f, 110.0f, -20.0f, 14.0f, 45.0f, 10.0f, 54.0f, 36.5f};

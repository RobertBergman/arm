// Per-arm calibration: edit this file after assembly.
#include "ArmConfig.h"

// Pulse-width change per degree of servo rotation.
//   DS3218MG 270-degree version: 500..2500 us over 270 deg
//   MG90S:                       ~500..2400 us over 180 deg
const float US_PER_DEG_STD = 2000.0f / 270.0f;
const float US_PER_DEG_MICRO = 1900.0f / 180.0f;

// The sign is the direction the JOINT moves when the pulse gets longer; it
// follows from how each servo is mounted.  If a joint moves the wrong way on
// your servos, flip the sign.  centerUs is the pulse at the zero pose: adjust
// it with the "trim" serial command, then copy the value printed by "show".
//
//                     channel  centerUs  us/deg             limits (deg)   speed (deg/s)
ServoCalibration ARM_CALIBRATION[JOINT_COUNT] = {
    /* turret      */ {0, 1500, +US_PER_DEG_STD, {-135.0f, 135.0f}, 60.0f},
    /* shoulder    */ {1, 1500, +US_PER_DEG_STD, {-30.0f, 90.0f}, 45.0f},
    /* elbow       */ {2, 1500, -US_PER_DEG_STD, {-90.0f, 45.0f}, 45.0f},
    /* wrist pitch */ {3, 1500, -US_PER_DEG_MICRO, {-90.0f, 30.0f}, 90.0f},
    /* wrist roll  */ {4, 1500, -US_PER_DEG_MICRO, {-90.0f, 90.0f}, 120.0f},
    /* grip        */ {5, 1500, +US_PER_DEG_MICRO, {-7.0f, 12.0f}, 40.0f},
};

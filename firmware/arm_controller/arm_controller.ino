// 6-servo desktop arm controller.
//
// Hardware: any Arduino-compatible board with I2C (tested layout: ESP32 DevKit)
//           + PCA9685 servo driver.  See docs/WIRING.md.
// Library:  "Adafruit PWM Servo Driver Library" (Arduino Library Manager).
//
// Serial commands (115200 baud, newline terminated):
//   home               go to the zero pose (use this pose to fit the horns)
//   j <0-5> <deg>      move one joint (0 turret, 1 shoulder, 2 elbow,
//                      3 wrist pitch, 4 wrist roll, 5 finger angle)
//   grip <mm>          set the jaw opening
//   go <x> <y> <z>     put the grip pads at a grid position, tool vertical
//   run                pick at zone A, place at zone B
//   trim <0-5> <us>    shift a joint's zero by <us> microseconds
//   show               print joint angles and calibration
//   off                release all servos
#include <Arduino.h>

#include "ArmConfig.h"
#include "JointController.h"
#include "Kinematics.h"
#include "Pca9685Output.h"
#include "TransferRoutine.h"

const unsigned long UPDATE_PERIOD_MS = 20;

Pca9685Output servoBoard;
JointController joints(servoBoard, ARM_CALIBRATION);
Kinematics kinematics(ARM_GEOMETRY);

void runUntilSettled(JointController& controller) {
  while (controller.update(UPDATE_PERIOD_MS / 1000.0f)) delay(UPDATE_PERIOD_MS);
}

TransferRoutine transfer(kinematics, ARM_GEOMETRY, joints, TRANSFER, runUntilSettled);

Pose zeroPose() {
  Pose pose;
  for (int j = 0; j < JOINT_COUNT; ++j) pose.q[j] = 0.0f;
  return pose;
}

void show() {
  for (int j = 0; j < JOINT_COUNT; ++j) {
    Serial.print(F("joint "));
    Serial.print(j);
    Serial.print(F(": "));
    Serial.print(joints.current().q[j], 1);
    Serial.print(F(" deg, centerUs "));
    Serial.println(ARM_CALIBRATION[j].centerUs);
  }
}

void handle(const String& line) {
  // Split "<command> <a> <b> <c>" by hand: scanf("%f") is missing on AVR boards.
  char buffer[48];
  line.toCharArray(buffer, sizeof(buffer));
  const char* word = strtok(buffer, " \t");
  const String name(word ? word : "");
  float args[3] = {0, 0, 0};
  int n = 1;
  for (; n < 4; ++n) {
    const char* token = strtok(NULL, " \t");
    if (!token) break;
    args[n - 1] = atof(token);
  }
  const float a = args[0], b = args[1], c = args[2];
  const int joint = static_cast<int>(a);
  const bool jointOk = joint >= 0 && joint < JOINT_COUNT;

  if (name == "home") {
    joints.setTarget(zeroPose());
  } else if (name == "j" && n == 3 && jointOk) {
    joints.setTarget(static_cast<Joint>(joint), b);
  } else if (name == "grip" && n == 2) {
    joints.setTarget(JOINT_GRIP, kinematics.fingerAngle(a));
  } else if (name == "go" && n == 4) {
    Pose pose = joints.current();
    if (kinematics.solve(a, b, c, 0.0f, ARM_GEOMETRY.gripNominal, pose)) {
      pose.q[JOINT_GRIP] = joints.current().q[JOINT_GRIP];
      joints.setTarget(pose);
    } else {
      Serial.println(F("out of reach"));
      return;
    }
  } else if (name == "run") {
    Serial.println(transfer.run() ? F("transfer done") : F("transfer: out of reach"));
    return;
  } else if (name == "trim" && n == 3 && jointOk) {
    joints.trim(static_cast<Joint>(joint), static_cast<int>(b));
  } else if (name == "show") {
    show();
    return;
  } else if (name == "off") {
    joints.releaseAll();
    return;
  } else {
    Serial.println(F("? commands: home, j, grip, go, run, trim, show, off"));
    return;
  }
  runUntilSettled(joints);
  Serial.println(F("ok"));
}

void setup() {
  Serial.begin(115200);
  joints.begin(zeroPose());  // every servo to its centre: the assembly pose
  Serial.println(F("arm ready - type 'home' or 'run'"));
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length()) handle(line);
  }
}

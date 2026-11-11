// Host-side test of the hardware-independent firmware code.
// Build and run with:  make -C firmware/test
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "ArmConfig.h"
#include "JointController.h"
#include "Kinematics.h"
#include "TransferRoutine.h"

struct Record {
  unsigned char channel;
  unsigned int us;
};

class RecordingOutput : public ServoOutput {
 public:
  void begin() override {}
  void writeMicroseconds(unsigned char channel, unsigned int us) override {
    log.push_back({channel, us});
  }
  void release(unsigned char) override {}
  std::vector<Record> log;
};

static int failures = 0;
#define CHECK(cond, ...)            \
  do {                              \
    if (!(cond)) {                  \
      ++failures;                   \
      std::printf("FAIL: ");        \
      std::printf(__VA_ARGS__);     \
      std::printf("\n");            \
    }                               \
  } while (0)

// Forward kinematics, written independently of Kinematics::solve.
static void padCentre(const Pose& p, float& x, float& y, float& z) {
  const float d2r = 3.14159265f / 180.0f;
  const ArmGeometry& g = ARM_GEOMETRY;
  const float s = p.q[JOINT_SHOULDER] * d2r, e = (p.q[JOINT_SHOULDER] + p.q[JOINT_ELBOW]) * d2r;
  const float reach = g.upperArm * std::sin(s) + g.forearm * std::cos(e);
  z = g.shoulderHeight + g.upperArm * std::cos(s) - g.forearm * std::sin(e) - g.toolLength;
  x = g.baseX + reach * std::sin(p.q[JOINT_TURRET] * d2r);
  y = g.baseY - reach * std::cos(p.q[JOINT_TURRET] * d2r);
}

static void settle(JointController& c) {
  int guard = 0;
  while (c.update(0.02f)) CHECK(++guard < 100000, "motion never settles");
}

int main(int argc, char**) {
  Kinematics kin(ARM_GEOMETRY);

  // 1. inverse kinematics round-trips through forward kinematics
  const float targets[][3] = {{-110, -20, 14}, {110, -20, 14}, {0, -74, 69}, {-110, -20, 59},
                              {60, -60, 30},   {0, -110, 14}};
  for (const auto& t : targets) {
    Pose p;
    CHECK(kin.solve(t[0], t[1], t[2], 0, 38, p), "target unreachable");
    float x, y, z;
    padCentre(p, x, y, z);
    CHECK(std::fabs(x - t[0]) < 0.05f && std::fabs(y - t[1]) < 0.05f && std::fabs(z - t[2]) < 0.05f,
          "FK(IK) mismatch at (%g, %g, %g): got (%g, %g, %g)", t[0], t[1], t[2], x, y, z);
    CHECK(std::fabs(p.q[JOINT_WRIST_PITCH] + p.q[JOINT_SHOULDER] + p.q[JOINT_ELBOW]) < 1e-3f,
          "tool not vertical");
    CHECK(std::fabs(p.q[JOINT_WRIST_ROLL] + p.q[JOINT_TURRET]) < 1e-3f, "heading not held");
  }
  Pose unreachable;
  CHECK(!kin.solve(400, 0, 0, 0, 38, unreachable), "far target should be rejected");

  // 2. the whole transfer stays inside joint limits and valid pulse widths
  RecordingOutput output;
  JointController joints(output, ARM_CALIBRATION);
  Pose zero;
  for (float& q : zero.q) q = 0;
  joints.begin(zero);
  for (int j = 0; j < JOINT_COUNT; ++j)
    CHECK(output.log[j].us == 1500, "zero pose must be 1500 us on every channel");

  TransferRoutine transfer(kin, ARM_GEOMETRY, joints, TRANSFER, settle);
  for (int i = 0; i <= 20; ++i) {
    Pose p;
    CHECK(transfer.carryPose(i / 20.0f, TRANSFER.holdWidth, p), "carry path unreachable");
    for (int j = 0; j < JOINT_COUNT; ++j)
      CHECK(p.q[j] >= ARM_CALIBRATION[j].limit.minDeg && p.q[j] <= ARM_CALIBRATION[j].limit.maxDeg,
            "carry step %d: joint %d = %.1f deg is outside its limit", i, j, p.q[j]);
  }
  CHECK(transfer.run(), "transfer failed");
  float x, y, z;
  padCentre(joints.current(), x, y, z);
  CHECK(std::fabs(x - TRANSFER.placeX) < 0.1f && std::fabs(y - TRANSFER.placeY) < 0.1f,
        "transfer should end above zone B, ended at (%g, %g)", x, y);
  for (const Record& r : output.log)
    CHECK(r.us >= 600 && r.us <= 2400 && r.channel < JOINT_COUNT,
          "pulse %u us on channel %u is out of the safe range", r.us, r.channel);

  // 3. optional: print poses for cross-checking against the CAD kinematics
  if (argc > 1) {
    for (const auto& t : targets) {
      Pose p;
      kin.solve(t[0], t[1], t[2], 0, 36.5f, p);
      std::printf("%g %g %g %.4f %.4f %.4f %.4f %.4f %.4f\n", t[0], t[1], t[2], p.q[0], p.q[1],
                  p.q[2], p.q[3], p.q[4], p.q[5]);
    }
  }
  std::printf("%s (%zu servo writes checked)\n", failures ? "FAILED" : "all firmware tests passed",
              output.log.size());
  return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}

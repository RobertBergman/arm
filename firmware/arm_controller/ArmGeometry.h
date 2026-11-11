// Dimensions of the printed arm, in millimetres.
// Keep in sync with cad/armcad/layout.py (python cad/build.py prints them).
#pragma once

struct ArmGeometry {
  float baseX;          // turret axis position on the grid
  float baseY;
  float shoulderHeight; // shoulder axis above the grid
  float upperArm;       // shoulder axis -> elbow axis
  float forearm;        // elbow axis -> wrist pitch axis
  float toolLength;     // wrist pitch axis -> centre of the grip pads
  float gripNominal;    // jaw opening with the fingers parallel
  float padRadius;      // finger pivot -> pad centre
};

const ArmGeometry ARM_GEOMETRY = {0.0f, 65.0f, 71.5f, 105.0f, 105.0f, 102.1f, 38.0f, 50.0f};

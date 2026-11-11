# Host-side test of the firmware logic (no Arduino needed):  make -C firmware/test
SRC := ../arm_controller
OUT ?= /tmp/arm_firmware_test

test: $(OUT)
	$(OUT)

$(OUT): test_firmware.cpp $(SRC)/Kinematics.cpp $(SRC)/JointController.cpp \
        $(SRC)/TransferRoutine.cpp $(SRC)/ArmConfig.cpp $(wildcard $(SRC)/*.h)
	$(CXX) -std=c++11 -Wall -Wextra -I$(SRC) $(filter %.cpp,$^) -o $@

.PHONY: test

# Syntax-check the Arduino sketch itself against stub headers.
sketch-check:
	$(CXX) -std=c++11 -Wall -Wextra -fsyntax-only -Imocks -I$(SRC) -x c++ $(SRC)/arm_controller.ino

.PHONY: sketch-check

#!/usr/bin/env python3
import os
import sys
import time

sys.path.insert(0, os.path.expanduser("~/RoboMaster-SDK/src"))

from robomaster import robot

# ===== Initial arm pose after recenter =====
INIT_X_MM = 120
INIT_Y_MM = 120
INIT_SETTLE_TIME = 1.0

# ===== Lua parameters converted to real robot mm / degrees =====
EXTRA_FORWARD_MM = 30

COARSE_FORWARD_MM = 60
COARSE_DOWN_MM = -240

FINAL_FORWARD_MM = 6
FINAL_DOWN_MM = -240

TEST_LIFT_MM = 25
MAIN_LIFT_MM = 60
RELEASE_DOWN_MM = -80
FINAL_LIFT_MM = 70

OPEN_POWER = 35
GRIP_POWER = 60
GRIP_CLOSE_PULSE_TIME = 1.5
GRIP_PAUSE_TIME = 1.0

RIGHT_TURN_DEG = -120
TURN_BACK_DEG = 90
TURN_SPEED_DPS = 20

OPEN_TIME = 1.5
CHASSIS_SETTLE_TIME = 1.2
FORWARD_MOVE_TIME = 6.0
FORWARD_SETTLE_TIME = 1.0
COARSE_MOVE_TIME = 9.0
COARSE_SETTLE_TIME = 1.0
FINAL_MOVE_TIME = 5.0
GRASP_HEIGHT_PAUSE_TIME = 1.2
TEST_LIFT_TIME = 5.0
TEST_SETTLE_TIME = 1.0
MAIN_LIFT_TIME = 7.0
BEFORE_TURN_TIME = 1.5
AFTER_TURN_TIME = 1.5
LOWER_TIME = 7.0
GROUND_PAUSE_TIME = 1.2
RELEASE_TIME = 2.5
FINAL_LIFT_TIME = 6.0

def step(label, sec=0.0):
    print("\n========================================")
    print(label)
    print("========================================")
    if sec > 0:
        time.sleep(sec)

def move_arm_delta(arm, dx_mm, dy_mm, label, wait_time):
    print(f"{label}: arm.move x={dx_mm} mm, y={dy_mm} mm")
    arm.move(x=dx_mm, y=dy_mm).wait_for_completed()
    time.sleep(wait_time)

def main():
    print("Connecting RoboMaster...")
    ep = robot.Robot()
    ep.initialize(conn_type="ap")

    arm = ep.robotic_arm
    gripper = ep.gripper
    chassis = ep.chassis

    try:
        step("STEP 0A: RECENTER ARM")
        arm.recenter().wait_for_completed()
        time.sleep(2)

        step("STEP 0B: MOVE TO INITIAL ARM POSE")
        print(f"initial pose: arm.moveto x={INIT_X_MM} mm, y={INIT_Y_MM} mm")
        arm.moveto(x=INIT_X_MM, y=INIT_Y_MM).wait_for_completed()
        time.sleep(INIT_SETTLE_TIME)

        step("STEP 1: OPEN GRIPPER")
        gripper.open(power=OPEN_POWER)
        time.sleep(OPEN_TIME)

        step("STEP 2: CHASSIS SETTLE")
        time.sleep(CHASSIS_SETTLE_TIME)

        step("STEP 3: FIRST FORWARD ALIGNMENT")
        move_arm_delta(arm, EXTRA_FORWARD_MM, 0, "forward alignment", FORWARD_MOVE_TIME)

        step("STEP 4: FORWARD SETTLE")
        time.sleep(FORWARD_SETTLE_TIME)

        step("STEP 5: FORWARD-LEANING COARSE DESCENT")
        move_arm_delta(arm, COARSE_FORWARD_MM, COARSE_DOWN_MM, "lean forward and down", COARSE_MOVE_TIME)

        step("STEP 6: PAUSE BEFORE FINAL APPROACH")
        time.sleep(COARSE_SETTLE_TIME)

        step("STEP 7: FINAL FORWARD-LEANING DESCENT")
        move_arm_delta(arm, FINAL_FORWARD_MM, FINAL_DOWN_MM, "final forward and down", FINAL_MOVE_TIME)

        step("STEP 8: AT GRASP POSITION")
        time.sleep(GRASP_HEIGHT_PAUSE_TIME)

        step("STEP 9: CLOSE GRIPPER")
        gripper.close(power=GRIP_POWER)
        time.sleep(GRIP_CLOSE_PULSE_TIME)

        step("STEP 10: PAUSE GRIPPER")
        time.sleep(GRIP_PAUSE_TIME)

        step("STEP 11: TEST LIFT")
        move_arm_delta(arm, 0, TEST_LIFT_MM, "small test lift", TEST_LIFT_TIME)

        step("STEP 12: CHECK REAL GRASP")
        print("Check visually whether the cube moved with the gripper.")
        time.sleep(TEST_SETTLE_TIME)

        step("STEP 13: MAIN LIFT")
        move_arm_delta(arm, 0, MAIN_LIFT_MM, "main lift", MAIN_LIFT_TIME)

        step("STEP 14: SETTLE BEFORE TURN")
        time.sleep(BEFORE_TURN_TIME)

        step("STEP 15: RIGHT TURN 90 DEG")
        chassis.move(x=0, y=0, z=RIGHT_TURN_DEG, z_speed=TURN_SPEED_DPS).wait_for_completed()
        time.sleep(AFTER_TURN_TIME)

        step("STEP 16: LOWER CUBE")
        move_arm_delta(arm, 0, RELEASE_DOWN_MM, "lower cube", LOWER_TIME)

        step("STEP 17: GROUND SETTLE")
        time.sleep(GROUND_PAUSE_TIME)

        step("STEP 18: RELEASE CUBE")
        gripper.open(power=OPEN_POWER)
        time.sleep(RELEASE_TIME)

        step("STEP 19: LIFT ARM AWAY")
        move_arm_delta(arm, 0, FINAL_LIFT_MM, "lift away", FINAL_LIFT_TIME)

        step("STEP 20: TURN CHASSIS BACK")
        chassis.move(x=0, y=0, z=TURN_BACK_DEG, z_speed=TURN_SPEED_DPS).wait_for_completed()
        time.sleep(2)

        step("STEP 21: DONE")
        gripper.open(power=OPEN_POWER)
        print("Pick-place sequence finished.")

    finally:
        print("Safe cleanup: open gripper and recenter arm")
        try:
            gripper.open(power=OPEN_POWER)
            time.sleep(1)
            arm.recenter().wait_for_completed()
        except Exception as e:
            print("Cleanup warning:", e)
        ep.close()

if __name__ == "__main__":
    main()

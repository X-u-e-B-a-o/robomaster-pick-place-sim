#!/usr/bin/env python3
import time

# 注意: 使用 pip 安装的官方 SDK (pip3 install --user ./RoboMaster-SDK)。
# 不要往 sys.path 插 ~/RoboMaster-SDK/src —— 板子上那份源码树版本混乱
# (client.py 引用不存在的 config.DEFAULT_CONN_PROTO), 会导致连接失败。
from robomaster import led
from robomaster import robot

RUN_COUNT = 5

INIT_X_MM = 120
INIT_Y_MM = 120
INIT_SETTLE_TIME = 1.0

EXTRA_FORWARD_MM = 30
COARSE_FORWARD_MM = 60
COARSE_DOWN_MM = -240
FINAL_FORWARD_MM = 6
FINAL_DOWN_MM = -24

TEST_LIFT_MM = 25
MAIN_LIFT_MM = 60
RELEASE_DOWN_MM = -80
FINAL_LIFT_MM = 70

OPEN_POWER = 35
GRIP_POWER = 60
GRIP_CLOSE_PULSE_TIME = 1.5
GRIP_PAUSE_TIME = 1.0

RIGHT_TURN_DEG = -180
TURN_SPEED_DPS = 20

OPEN_TIME = 1.5
CHASSIS_SETTLE_TIME = 1.2
FORWARD_MOVE_TIME = 1.0
FORWARD_SETTLE_TIME = 1.0
COARSE_MOVE_TIME = 1.0
COARSE_SETTLE_TIME = 1.0
FINAL_MOVE_TIME = 1.5
GRASP_HEIGHT_PAUSE_TIME = 1.2
TEST_LIFT_TIME = 1.5
TEST_SETTLE_TIME = 1.0
MAIN_LIFT_TIME = 1.5
BEFORE_TURN_TIME = 1.5
AFTER_TURN_TIME = 1.5
LOWER_TIME = 1.5
GROUND_PAUSE_TIME = 1.2
RELEASE_TIME = 1.5
FINAL_LIFT_TIME = 1.5

RETRY_PAUSE_TIME = 2.0

gripper_status = {"value": "unknown"}

# ROS2 包装节点(real_pick_place_ros2_node.py)会注入发布函数,
# 使 ERROR/SUCCESS 状态同步显示到 /real_pick_place/status 话题。
status_hook = None


def report_status(text):
    print(text)
    if status_hook is not None:
        try:
            status_hook(text)
        except Exception:
            pass


def step(attempt, number, label, sec=0.0):
    print("\n========================================")
    print(f"RUN {attempt}/{RUN_COUNT} | STEP {number}: {label}")
    print("========================================")
    if sec > 0:
        time.sleep(sec)


def on_gripper_status(status):
    if isinstance(status, (list, tuple)) and status:
        status = status[0]
    gripper_status["value"] = status


def is_fully_closed(status):
    """判断夹爪是否完全闭合。

    DJI SDK gripper.sub_status 回调返回字符串:
        "closed"  夹爪完全闭合 -> 说明中间没有物体(未探测到东西)
        "opened"  夹爪完全张开
        "normal"  处在中间位置 -> 说明夹到了物体
    这里同时兼容历史测试中直接传 int 的情况(2 = closed)。
    """
    if isinstance(status, (list, tuple)) and status:
        status = status[0]

    if isinstance(status, int):
        return status == 2

    text = str(status).lower()
    if text in ("2", "closed", "close", "fully_closed"):
        return True
    if "closed" in text and "opened" not in text:
        return True
    return False


def move_arm_delta(arm, dx_mm, dy_mm, label, wait_time):
    print(f"{label}: arm.move x={dx_mm} mm, y={dy_mm} mm")
    arm.move(x=dx_mm, y=dy_mm).wait_for_completed()
    time.sleep(wait_time)


def safe_home(arm, gripper):
    print("\n========================================")
    print("SAFE HOME: open gripper and recenter arm")
    print("========================================")
    try:
        gripper.open(power=OPEN_POWER)
        time.sleep(1.0)
    except Exception as e:
        print("Open gripper warning:", e)

    try:
        arm.recenter().wait_for_completed()
        time.sleep(2.0)
    except Exception as e:
        print("Arm recenter warning:", e)


def robot_signal_error(ep):
    """抓取失败报错显示: 全车装甲灯红色闪烁 + 警报音效。"""
    try:
        ep.led.set_led(
            comp=led.COMP_ALL, r=255, g=0, b=0,
            effect=led.EFFECT_FLASH, freq=5,
        )
    except Exception as e:
        print("Set error LED warning:", e)
    try:
        ep.play_sound(robot.SOUND_ID_ATTACK).wait_for_completed(timeout=3)
    except Exception as e:
        print("Play error sound warning:", e)


def robot_signal_success(ep):
    """抓取成功显示: 全车装甲灯绿色常亮 + 成功音效。"""
    try:
        ep.led.set_led(
            comp=led.COMP_ALL, r=0, g=255, b=0,
            effect=led.EFFECT_ON,
        )
    except Exception as e:
        print("Set success LED warning:", e)
    try:
        ep.play_sound(robot.SOUND_ID_RECOGNIZED).wait_for_completed(timeout=3)
    except Exception as e:
        print("Play success sound warning:", e)


def robot_signal_idle(ep):
    """熄灭装甲灯, 恢复默认状态。"""
    try:
        ep.led.set_led(comp=led.COMP_ALL, effect=led.EFFECT_OFF)
    except Exception as e:
        print("Set idle LED warning:", e)


def initialize_arm(attempt, arm, gripper):
    step(attempt, "0A", "RECENTER ARM")
    arm.recenter().wait_for_completed()
    time.sleep(2.0)

    step(attempt, "0B", "MOVE TO INITIAL ARM POSE")
    print(f"initial pose: arm.moveto x={INIT_X_MM} mm, y={INIT_Y_MM} mm")
    arm.moveto(x=INIT_X_MM, y=INIT_Y_MM).wait_for_completed()
    time.sleep(INIT_SETTLE_TIME)

    step(attempt, 1, "OPEN GRIPPER")
    gripper.open(power=OPEN_POWER)
    time.sleep(OPEN_TIME)


def run_once(attempt, ep, arm, gripper, chassis):
    initialize_arm(attempt, arm, gripper)

    step(attempt, 2, "CHASSIS SETTLE")
    time.sleep(CHASSIS_SETTLE_TIME)

    step(attempt, 3, "FIRST FORWARD ALIGNMENT")
    move_arm_delta(arm, EXTRA_FORWARD_MM, 0, "forward alignment", FORWARD_MOVE_TIME)

    step(attempt, 4, "FORWARD SETTLE")
    time.sleep(FORWARD_SETTLE_TIME)

    step(attempt, 5, "FORWARD-LEANING COARSE DESCENT")
    move_arm_delta(arm, COARSE_FORWARD_MM, COARSE_DOWN_MM, "lean forward and down", COARSE_MOVE_TIME)

    step(attempt, 6, "PAUSE BEFORE FINAL APPROACH")
    time.sleep(COARSE_SETTLE_TIME)

    step(attempt, 7, "FINAL FORWARD-LEANING DESCENT")
    move_arm_delta(arm, FINAL_FORWARD_MM, FINAL_DOWN_MM, "final forward and down", FINAL_MOVE_TIME)

    step(attempt, 8, "AT GRASP POSITION")
    time.sleep(GRASP_HEIGHT_PAUSE_TIME)

    step(attempt, 9, "CLOSE GRIPPER")
    gripper_status["value"] = "unknown"
    gripper.close(power=GRIP_POWER)
    time.sleep(GRIP_CLOSE_PULSE_TIME)
    try:
        gripper.pause()
    except Exception:
        pass
    time.sleep(GRIP_PAUSE_TIME)

    step(attempt, 10, "CHECK GRIPPER CLOSED ANGLE / STATUS")
    current_status = gripper_status["value"]
    print(f"gripper status after close: {current_status}")

    # 关键判定: 夹爪完全闭合 => 未探测到物体 => 报错并退出循环
    if is_fully_closed(current_status):
        report_status(
            f"ERROR: RUN {attempt} 抓取失败 - 夹爪完全闭合, 未探测到物体, 停止循环"
        )
        robot_signal_error(ep)
        safe_home(arm, gripper)
        return False

    # 订阅失败等异常情况拿不到状态, 无法判定 -> 按失败处理, 避免盲目继续
    if str(current_status).strip().lower() in ("", "unknown", "none"):
        report_status(
            f"ERROR: RUN {attempt} 无法读取夹爪状态({current_status}), 无法判定, 停止循环"
        )
        robot_signal_error(ep)
        safe_home(arm, gripper)
        return False

    report_status(
        f"SUCCESS: RUN {attempt} 夹爪未完全闭合({current_status}), 判定已夹到物体"
    )
    robot_signal_success(ep)

    step(attempt, 11, "TEST LIFT")
    move_arm_delta(arm, 0, TEST_LIFT_MM, "small test lift", TEST_LIFT_TIME)

    step(attempt, 12, "CHECK REAL GRASP")
    print("Check visually whether the cube moved with the gripper.")
    time.sleep(TEST_SETTLE_TIME)

    step(attempt, 13, "MAIN LIFT")
    move_arm_delta(arm, 0, MAIN_LIFT_MM, "main lift", MAIN_LIFT_TIME)

    step(attempt, 14, "SETTLE BEFORE TURN")
    time.sleep(BEFORE_TURN_TIME)

    step(attempt, 15, "RIGHT TURN TO PLACE AREA")
    chassis.move(x=0, y=0, z=RIGHT_TURN_DEG, z_speed=TURN_SPEED_DPS).wait_for_completed()
    time.sleep(AFTER_TURN_TIME)

    step(attempt, 16, "LOWER CUBE AT B POINT")
    move_arm_delta(arm, 0, RELEASE_DOWN_MM, "lower cube", LOWER_TIME)

    step(attempt, 17, "GROUND SETTLE")
    time.sleep(GROUND_PAUSE_TIME)

    step(attempt, 18, "RELEASE CUBE")
    gripper.open(power=OPEN_POWER)
    time.sleep(RELEASE_TIME)

    step(attempt, 19, "LIFT ARM AWAY")
    move_arm_delta(arm, 0, FINAL_LIFT_MM, "lift away", FINAL_LIFT_TIME)

    step(attempt, 20, "DONE - KEEP FINAL POSE")
    print("This run finished. No chassis turn-back. Next run will initialize arm again.")
    report_status(f"SUCCESS: RUN {attempt} 抓取-放置完成")
    robot_signal_idle(ep)
    return True


def main():
    print("Connecting RoboMaster...")
    ep = robot.Robot()
    ep.initialize(conn_type="ap")

    arm = ep.robotic_arm
    gripper = ep.gripper
    chassis = ep.chassis

    success_count = 0
    fail_count = 0

    try:
        try:
            gripper.sub_status(freq=5, callback=on_gripper_status)
            time.sleep(0.5)
        except Exception as e:
            print("Gripper status subscribe warning:", e)

        for attempt in range(1, RUN_COUNT + 1):
            ok = run_once(attempt, ep, arm, gripper, chassis)

            if not ok:
                fail_count += 1
                print("Loop stopped because grasp failed.")
                break

            success_count += 1

            if attempt < RUN_COUNT:
                print(f"Run {attempt} success. Prepare object, then next run starts.")
                time.sleep(RETRY_PAUSE_TIME)

        print("\n========================================")
        print("FINAL RESULT")
        print("========================================")
        print(f"success: {success_count}")
        print(f"failed: {fail_count}")
        print(f"planned runs: {RUN_COUNT}")

        # 最终判定: 失败保持红灯闪烁报错, 全部成功亮绿灯报成功
        if fail_count > 0:
            report_status(
                f"ERROR: 抓取失败, 循环已停止 "
                f"(success={success_count}, failed={fail_count}, planned={RUN_COUNT})"
            )
            robot_signal_error(ep)
        else:
            report_status(f"SUCCESS: 全部 {success_count}/{RUN_COUNT} 次抓取成功")
            robot_signal_success(ep)

        return success_count, fail_count

    except KeyboardInterrupt:
        print("Interrupted by user.")
        safe_home(arm, gripper)
        return None

    except Exception as e:
        print("Unexpected error:", e)
        safe_home(arm, gripper)
        robot_signal_error(ep)
        raise

    finally:
        try:
            gripper.unsub_status()
        except Exception:
            pass
        ep.close()


if __name__ == "__main__":
    main()

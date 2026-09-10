#!/usr/bin/env python3
"""夹爪状态探针: 观察空夹闭合时的真实状态转换, 确认判定修复生效。

用法 (板子上, 已连机器人热点 RMEP-xxxx):
    cd ~/colcon_ws/src/robomaster_pick_place_sim
    python3 real/gripper_status_probe.py

流程: 张开 -> 观察 -> 闭合(不 pause) -> 每 0.5s 打印状态共 10s

预期:
- 空夹(爪子里没东西): 状态最终变成 closed(可能先经过 normal, 因为
  闭合过程中爪子处在中间位置)
- 爪子里放个方块再跑一次: 状态停在 normal

如果空夹时状态永远停在 normal, 说明夹爪走不到机械限位(可能功率不够
或行程被卡), 需要加大 GRIP_POWER 或排查机械问题。
"""
import time

from robomaster import robot


def main():
    ep = robot.Robot()
    ep.initialize(conn_type="ap")

    last = {"v": "unknown"}

    def cb(s):
        last["v"] = s

    ep.gripper.sub_status(freq=5, callback=cb)
    time.sleep(0.5)

    print("1) OPEN")
    ep.gripper.open(power=35)
    for i in range(10):
        print(f"  open  t={i*0.5:4.1f}s  status={last['v']}")
        time.sleep(0.5)

    print("2) CLOSE (no pause, let it travel to the limit)")
    ep.gripper.close(power=60)
    for i in range(20):
        print(f"  close t={i*0.5:4.1f}s  status={last['v']}")
        time.sleep(0.5)

    ep.gripper.unsub_status()
    ep.close()
    print("probe done: 空夹最终应为 closed; 夹着方块时停在 normal")


if __name__ == "__main__":
    main()

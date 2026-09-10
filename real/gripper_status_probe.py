#!/usr/bin/env python3
"""夹爪状态探针: 观察闭合时状态转换, 支持指定闭合功率。

用法 (板子上, 已连机器人热点):
    cd ~/colcon_ws/src/robomaster_pick_place_sim
    python3 real/gripper_status_probe.py [close_power]

默认 close_power=60。

现象回顾:
- 空夹 power=60: 中途经过 normal, 最终停在 closed (已确认, 正确报错)
- 夹着方块 power=60: 最终也变成 closed -> 误报"未夹到物体"
  (功率太大, 爪子把方块挤压进固件的"闭合区")

测试矩阵 (每步之间手动把爪子掰开/塞方块):
  1. 爪子里没东西, power 60  -> 预期最终 closed
  2. 爪子里塞方块, power 60  -> 记录最终停在什么
  3. 爪子里塞方块, power 30  -> 记录最终停在什么
  4. 爪子里塞方块, power 20  -> 记录最终停在什么

目标: 找到"空夹 -> closed, 夹方块 -> normal"的功率值,
然后把它设为主脚本的 GRIP_POWER。
"""
import subprocess
import sys
import time

from robomaster import robot


def current_wifi_ssid():
    try:
        out = subprocess.run(
            ["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"],
            capture_output=True, text=True, timeout=5,
        ).stdout
        for line in out.splitlines():
            if line.startswith("yes:"):
                return line.split(":", 1)[1]
    except Exception:
        return "?"
    return ""


def main():
    close_power = int(sys.argv[1]) if len(sys.argv) > 1 else 60

    ssid = current_wifi_ssid()
    print(f"current wifi: {ssid!r}")
    if not ssid.startswith("RMEP"):
        print("ERROR: 板子没连机器人热点, 不会开始测试!")
        print("       先开机机器人, 然后执行:")
        print("           nmcli connection up RMEP-21bbc5")
        sys.exit(1)

    ep = robot.Robot()
    try:
        ep.initialize(conn_type="ap")
    except Exception as e:
        print(f"ERROR: SDK 初始化失败(机器人没开机?): {e}")
        sys.exit(1)

    last = {"v": "unknown"}

    def cb(s):
        last["v"] = s

    try:
        ep.gripper.sub_status(freq=5, callback=cb)
        time.sleep(0.5)

        print("1) OPEN")
        ep.gripper.open(power=35)
        for i in range(8):
            print(f"  open  t={i*0.5:4.1f}s  status={last['v']}")
            time.sleep(0.5)

        print(f"2) CLOSE (power={close_power}, no pause)")
        ep.gripper.close(power=close_power)
        for i in range(24):
            print(f"  close t={i*0.5:4.1f}s  status={last['v']}")
            time.sleep(0.5)

        ep.gripper.unsub_status()
        print(f"probe done: power={close_power}, 最终状态={last['v']}")
        print("  空夹最终应为 closed; 夹着方块时应停在 normal")
    finally:
        try:
            ep.close()
        except Exception as e:
            print("close warning:", e)


if __name__ == "__main__":
    main()

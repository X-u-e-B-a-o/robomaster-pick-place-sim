#!/usr/bin/env python3
"""夹爪功率-状态扫描探针 (网球调试专用)。

背景: 网球是软的, 大功率会把球压进固件的"闭合区", status 变成 closed,
导致夹到球也报"未夹到物体"。本脚本一次跑完多个功率档, 找出
"空夹 -> closed, 夹球 -> normal" 的功率, 或收集时间特征做备选判据。

用法 (板子上, 已连机器人热点):
    cd ~/colcon_ws/src/robomaster_pick_place_sim
    python3 real/gripper_status_sweep.py

需要跑两遍:
  A. 爪子里不放东西
  B. 第一阶段的 OPEN 期间把网球塞进爪子并扶着, 直到 CLOSE 夹住它

每阶段: OPEN(功率60, 2.5s) -> CLOSE(指定功率, 8s), 每 0.5s 打印状态。
最后输出每个功率的汇总: 首次 normal 时间 / 首次 closed 时间 / 最终状态。

把 A、B 两遍最后的汇总表发给开发者, 用来确定判定功率或时间阈值。
"""
import subprocess
import sys
import time

from robomaster import robot

POWERS = [60, 30, 20, 10, 5]


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
    ssid = current_wifi_ssid()
    print(f"current wifi: {ssid!r}")
    if not ssid.startswith("RMEP"):
        print("ERROR: 板子没连机器人热点, 不会开始测试!")
        print("       先开机机器人, 然后执行: nmcli connection up RMEP-21bbc5")
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

    results = []

    try:
        ep.gripper.sub_status(freq=5, callback=cb)
        time.sleep(0.5)

        for power in POWERS:
            print(f"\n========== phase: CLOSE power={power} ==========")
            print("(OPEN 阶段 2.5s: 如果是 B 遍, 请现在把网球塞进爪子并扶着)")
            ep.gripper.open(power=60)
            for i in range(5):
                print(f"  open  t={i*0.5:4.1f}s  status={last['v']}")
                time.sleep(0.5)

            print("(CLOSE 开始, 扶着球的手可以松开了)")
            t1 = time.time()
            ep.gripper.close(power=power)
            first_normal = None
            first_closed = None
            for i in range(16):
                t = time.time() - t1
                v = str(last["v"])
                if v == "normal" and first_normal is None:
                    first_normal = round(t, 1)
                if v == "closed" and first_closed is None:
                    first_closed = round(t, 1)
                print(f"  close t={t:4.1f}s  status={v}")
                time.sleep(0.5)

            results.append((power, first_normal, first_closed, str(last["v"])))

        print("\n========== SUMMARY ==========")
        print("power | first_normal(s) | first_closed(s) | final")
        for power, fn, fc, final in results:
            print(f"  {power:>5} | {fn!s:>15} | {fc!s:>14} | {final}")
        print("目标: 找到 空夹->closed 且 夹球->normal 的功率;")
        print("      若没有, 对比 A/B 的 first_normal / first_closed 时间差异。")

        ep.gripper.unsub_status()
    finally:
        try:
            ep.close()
        except Exception as e:
            print("close warning:", e)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""夹爪功率-状态扫描探针 (网球调试专用)。

背景: 网球是软的, 大功率会把球压进固件的"闭合区", status 变成 closed,
导致夹到球也报"未夹到物体"。本脚本一次跑完多个功率档, 找出
"空夹 -> closed, 夹球 -> normal" 的功率, 或收集时间特征做备选判据。

用法 (板子上, 已连机器人热点):
    cd ~/colcon_ws/src/robomaster_pick_place_sim
    python3 real/gripper_status_sweep.py [标记]

标记建议用 empty / ball 区分两遍:
  A 遍: python3 real/gripper_status_sweep.py empty   (爪子里不放东西)
  B 遍: python3 real/gripper_status_sweep.py ball    (OPEN 阶段把网球塞进爪子并扶着)

每阶段: OPEN(功率60, 2.5s) -> CLOSE(指定功率, 8s), 每 0.5s 打印状态。
完整输出 (含最后的汇总表) 自动保存到:
    ~/Desktop/gripper_sweep_<标记>_<时间戳>.txt
"""
import os
import subprocess
import sys
import time

from robomaster import robot

POWERS = [60, 30, 20, 10, 5]

LOG_LINES = []


def log(text):
    print(text)
    LOG_LINES.append(text)


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


def save_log(label):
    """把 LOG_LINES 存到桌面, 无论跑完还是报错都会调用。"""
    try:
        desktop = os.path.expanduser("~/Desktop")
        os.makedirs(desktop, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(desktop, f"gripper_sweep_{label}_{stamp}.txt")
        with open(path, "w") as f:
            f.write("\n".join(LOG_LINES) + "\n")
        print(f"日志已保存: {path}")
    except Exception as e:
        print(f"保存日志失败: {e}")


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else "run"
    log(f"gripper sweep label: {label}")

    ep = None
    try:
        ssid = current_wifi_ssid()
        log(f"current wifi: {ssid!r}")
        if not ssid.startswith("RMEP"):
            log("ERROR: 板子没连机器人热点, 不会开始测试!")
            log("       先开机机器人, 然后执行: nmcli connection up RMEP-21bbc5")
            sys.exit(1)

        ep = robot.Robot()
        try:
            ep.initialize(conn_type="ap")
        except Exception as e:
            log(f"ERROR: SDK 初始化失败(机器人没开机?): {e}")
            sys.exit(1)

        last = {"v": "unknown"}

        def cb(s):
            last["v"] = s

        results = []

        ep.gripper.sub_status(freq=5, callback=cb)
        time.sleep(0.5)

        for power in POWERS:
            log(f"\n========== phase: CLOSE power={power} ==========")
            log("(OPEN 阶段 2.5s: 如果是 ball 遍, 请现在把网球塞进爪子并扶着)")
            ep.gripper.open(power=60)
            for i in range(5):
                log(f"  open  t={i*0.5:4.1f}s  status={last['v']}")
                time.sleep(0.5)

            log("(CLOSE 开始, 扶着球的手可以松开了)")
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
                log(f"  close t={t:4.1f}s  status={v}")
                time.sleep(0.5)

            results.append((power, first_normal, first_closed, str(last["v"])))

        log("\n========== SUMMARY ==========")
        log("power | first_normal(s) | first_closed(s) | final")
        for power, fn, fc, final in results:
            log(f"  {power:>5} | {fn!s:>15} | {fc!s:>14} | {final}")
        log("目标: 找到 空夹->closed 且 夹球->normal 的功率;")
        log("      若没有, 对比 empty/ball 的 first_normal / first_closed 时间差异。")

        ep.gripper.unsub_status()
    finally:
        # 无论成功还是中途报错, 都保存日志再关连接
        save_log(label)
        if ep is not None:
            try:
                ep.close()
            except Exception as e:
                print("close warning:", e)


if __name__ == "__main__":
    main()

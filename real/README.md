# Real Robot Pick-Place

This folder contains the real RoboMaster EP/Core pick-place experiment files.

## Files

- real/pick_place_lua_params.py: real robot pick-place script with tuned parameters
- real/real_pick_place_ros2_node.py: ROS2 wrapper node
- launch/real_pick_place.launch.py: ROS2 launch file

## Before Running

0. One-time setup: `libmedia_codec` stub (aarch64 has no `.so`; the SDK's
   `media.py` instantiates the decoders, so the stub MUST provide both
   classes — a comment-only file fails with `no attribute H264Decoder`):

    mkdir -p ~/.local/lib/python3.10/site-packages
    cat > ~/.local/lib/python3.10/site-packages/libmedia_codec.py << 'EOF'
    class H264Decoder:
        def __init__(self, *a, **k): pass
        def decode(self, d): return []

    class OpusDecoder:
        def __init__(self, *a, **k): pass
        def decode(self, d): return []
    EOF

1. Power on RoboMaster.
2. Manually connect Jetson to RoboMaster Wi-Fi.
3. Check connection:

    ping -c 2 192.168.2.1

## Run Directly

    cd ~/colcon_ws/src/robomaster_pick_place_sim
    python3 real/pick_place_lua_params.py

The script uses the pip-installed official SDK (`pip3 install --user ./RoboMaster-SDK`).
Do NOT prepend `~/RoboMaster-SDK/src` to PYTHONPATH — that source tree on the
Jetson is a mixed-version snapshot whose `client.py` references a nonexistent
`config.DEFAULT_CONN_PROTO` and breaks the connection.

## Run With ROS2

真机节点已打包进 ROS2 包 (entry point `real_pick_place`)。
改动代码后先重新构建 (symlink 安装, 源码改动即时生效):

    cd ~/colcon_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select robomaster_pick_place_sim
    source install/setup.bash

然后二选一:

    ros2 run robomaster_pick_place_sim real_pick_place
    ros2 launch robomaster_pick_place_sim real_pick_place.launch.py

## Watch ROS2 Status

Open another terminal:

    source /opt/ros/humble/setup.bash
    ros2 topic echo /real_pick_place/status

## 运行日志 (桌面)

每次运行主脚本, 完整输出 (每一步动作、判定结果、出错回溯) 都会自动
保存到桌面 `~/Desktop/test_<时间戳>.txt` (如 `test_20260911_213045.txt`),
无论成败都会保存, 实验报告直接引用该文件。功率扫描的日志同理保存为
`~/Desktop/gripper_sweep_<标记>_<时间戳>.txt`。

## Behavior

The program runs up to 5 pick-place attempts.

Grasp judgment (DJI SDK gripper status, sub_status at 5 Hz):

The status alone cannot tell "object held" from "empty" — a soft object
(tennis ball) gets compressed into the firmware's `closed` zone, and a
low-power empty close can stall mid-travel at `normal`. The script therefore
judges by the TIME it takes to reach `closed` (`close_and_judge`):

- `closed` arrives within `GRIP_CLOSED_FAST` (2.0 s) of the close command
  -> jaws stopped early by an object -> grasp SUCCEEDED
  (measured: tennis ball at power 30 -> closed in ~1.5 s)
- `closed` arrives later than 2.0 s
  -> jaws ran the full stroke to the mechanical limit -> empty, FAILED
  (measured: empty at power 30 -> closed in ~2.5 s)
- status stays `normal` for `GRIP_STABLE_TIME` (2.5 s) with fresh pushes
  and never reaches `closed` -> rigid object (cube) holds the jaws open
  mid-stroke -> SUCCEEDED
- anything else (timeout `GRIP_STATUS_TIMEOUT` = 8 s, unreadable status,
  no pushes) -> FAILED

These numbers come from the on-robot power sweep
(`real/gripper_status_sweep.py`, saved to the desktop); power 30 gives the
largest empty/ball time gap (1.0 s). If you change objects, re-run the sweep
and re-tune `GRIP_POWER` / `GRIP_CLOSED_FAST`.

IMPORTANT: do NOT call `gripper.pause()` right after close. Pausing mid-close
freezes the jaws at a middle position, so an EMPTY close is misread as
`normal` (false success). The close command stays active and the firmware
stops the jaws at the mechanical limit by itself.

Also, the script pre-checks that the board's WiFi is on the robot AP
(`nmcli`) and that `robot.initialize()` returns success before doing anything,
so a wrong-network run aborts with a clear message instead of the SDK's
cryptic `NoneType.is_alive` teardown error.

`real/gripper_status_probe.py` prints the live status transitions during a
close with a given power — use it to spot-check behavior.
`real/gripper_status_sweep.py` runs the full power sweep and saves the log
plus summary to `~/Desktop/gripper_sweep_<label>_<timestamp>.txt`.

If grasp succeeds:
- robot shows SUCCESS: armor LEDs solid green + success sound
- per-run status published to `/real_pick_place/status` as `SUCCESS: ...`
- place object at B point
- keep chassis orientation
- initialize arm again
- start next run
- after all runs succeed: final `SUCCESS: 全部 N/N 次抓取成功`, green LEDs stay on

If grasp fails (slow `closed` = empty, timeout, or status unreadable):
- robot shows ERROR: armor LEDs flashing red + alarm sound (keeps flashing)
- status published as `ERROR: ...` (ROS2 logger also logs at error level)
- loop stops immediately
- gripper opens
- arm returns home

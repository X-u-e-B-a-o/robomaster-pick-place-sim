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

    cd ~/colcon_ws
    source /opt/ros/humble/setup.bash
    source install/robomaster_pick_place_sim/share/robomaster_pick_place_sim/package.bash
    ros2 launch robomaster_pick_place_sim real_pick_place.launch.py

## Watch ROS2 Status

Open another terminal:

    source /opt/ros/humble/setup.bash
    ros2 topic echo /real_pick_place/status

## Behavior

The program runs up to 5 pick-place attempts.

Grasp judgment (DJI SDK gripper status, sub_status at 5 Hz):
- `closed`  = gripper fully closed -> no object detected -> grasp FAILED
- `opened`  = gripper fully open
- `normal`  = middle position -> object held -> grasp SUCCEEDED

If grasp succeeds:
- robot shows SUCCESS: armor LEDs solid green + success sound
- per-run status published to `/real_pick_place/status` as `SUCCESS: ...`
- place object at B point
- keep chassis orientation
- initialize arm again
- start next run
- after all runs succeed: final `SUCCESS: 全部 N/N 次抓取成功`, green LEDs stay on

If grasp fails (gripper fully closed, or status unreadable):
- robot shows ERROR: armor LEDs flashing red + alarm sound (keeps flashing)
- status published as `ERROR: ...` (ROS2 logger also logs at error level)
- loop stops immediately
- gripper opens
- arm returns home

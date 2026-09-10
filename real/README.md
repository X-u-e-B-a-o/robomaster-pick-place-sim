# Real Robot Pick-Place

This folder contains the real RoboMaster EP/Core pick-place experiment files.

## Files

- real/pick_place_lua_params.py: real robot pick-place script with tuned parameters
- real/real_pick_place_ros2_node.py: ROS2 wrapper node
- launch/real_pick_place.launch.py: ROS2 launch file

## Before Running

1. Power on RoboMaster.
2. Manually connect Jetson to RoboMaster Wi-Fi.
3. Check connection:

    ping -c 2 192.168.2.1

## Run Directly

    cd ~/colcon_ws/src/robomaster_pick_place_sim
    PYTHONPATH=$HOME/RoboMaster-SDK/src python3 real/pick_place_lua_params.py

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

If grasp succeeds:
- place object at B point
- keep chassis orientation
- initialize arm again
- start next run

If grasp fails:
- gripper is fully closed
- object is considered not in grasp range
- loop stops immediately
- gripper opens
- arm returns home

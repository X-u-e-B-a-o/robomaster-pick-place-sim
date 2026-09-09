# robomaster-pick-place-sim

RoboMaster EP/Core pick-and-place simulation package for the fixed-point
manipulator experiment.

The experiment goal is:

```text
home -> move to pick point A -> close gripper -> lift -> move to place point B
-> open gripper -> return home
```

The project currently keeps two simulation routes:

- ROS 2 Humble + Gazebo: package, URDF, controller config, world file, and a
  basic trajectory publisher.
- CoppeliaSim: a RoboMaster EP arm-only timed pick-and-place script for current
  demonstration and tuning.

## Repository Layout

```text
config/                         ROS 2 controller configuration
launch/                         Gazebo launch file
scripts/pick_place_trajectory.py ROS 2 trajectory publisher demo
scripts/coppeliasim_arm_only_pick_place.lua
                                CoppeliaSim RoboMaster EP pick-place script
urdf/                           simplified RoboMaster EP model
worlds/                         Gazebo pick-place world
docs/                           progress notes and run instructions
```

## Gazebo Route

Build the ROS 2 package on Jetson:

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robomaster_pick_place_sim
source install/setup.bash
```

Launch Gazebo:

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export DISPLAY=:1
export GZ_VERSION=fortress
ros2 launch robomaster_pick_place_sim pick_place_gazebo.launch.py
```

Run the ROS 2 trajectory demo:

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/robomaster_pick_place_sim/scripts/pick_place_trajectory.py
```

Check controller status:

```bash
ros2 control list_controllers
```

Expected active controllers:

```text
joint_state_broadcaster
arm_controller
gripper_controller
```

## CoppeliaSim Route

The current CoppeliaSim script is:

```text
scripts/coppeliasim_arm_only_pick_place.lua
```

It uses the CoppeliaSim RoboMaster plugin API:

```lua
local simRobomaster = require('simRobomaster')
```

Recommended scene name:

```text
robomaster_ep_pick_place.ttt
```

Basic usage:

1. Open the RoboMaster EP scene in CoppeliaSim.
2. Stop simulation before editing the script.
3. Select the scene script in the scene hierarchy.
4. Replace its content with `scripts/coppeliasim_arm_only_pick_place.lua`.
5. Save the scene, then press Play.

Current script behavior:

```text
open gripper -> home -> aim above cube -> lower -> close gripper -> lift
-> turn right 90 degrees -> lower -> release -> return home
```

Detailed CoppeliaSim notes are in
`docs/coppeliasim_pick_place_progress.md`.

## Current Status

- The RoboMaster ROS 2 Gazebo package has been created and committed.
- The Gazebo route still needs controller activation and stable physical grasp
  tuning.
- The CoppeliaSim route now has a timed RoboMaster EP pick-place script and a
  documented tuning workflow.
- The CoppeliaSim scene file itself is not committed here unless it is exported
  and checked for size/licensing.

## Suggested Next Work

1. Record one full CoppeliaSim run and note whether the cube stays in the
   gripper during lift and turn.
2. Tune `cubeX`, `aboveServo1`, and `lowerServo0` in the Lua script.
3. If real friction grasp is unstable, prepare a separate attach/detach demo
   script for classroom presentation.
4. Continue Gazebo controller debugging after the CoppeliaSim demo is stable.

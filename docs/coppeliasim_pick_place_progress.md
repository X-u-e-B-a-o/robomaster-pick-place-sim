# CoppeliaSim RoboMaster EP Pick-Place Progress

Date: 2026-09-09

## Goal

Use CoppeliaSim to demonstrate a fixed-point RoboMaster EP/Core manipulator
pick-and-place sequence for the mechanical arm experiment.

This route focuses on the required fixed-position grasping process, not visual
recognition. The mobile chassis can remain fixed or only rotate in place during
the demonstration.

## Current Result

The repository now includes the current CoppeliaSim script:

```text
scripts/coppeliasim_arm_only_pick_place.lua
```

The script creates or reuses:

- `target_cube`: the grasp target.
- `place_marker`: a green placement marker.

The script runs this timed sequence:

```text
open gripper
home servo 0
home servo 1
move gripper above cube
lower gripper
pause at pick point
close gripper
lift with servo 0
lift with servo 1
turn right 90 degrees
lower for release
open gripper
return servo 0 home
return servo 1 home
```

This version is a real-physics attempt. It does not fake the grasp by attaching
the cube to the gripper. The cube only moves if the gripper clamps it through
CoppeliaSim physics.

## CoppeliaSim Scene

Recommended scene file name:

```text
robomaster_ep_pick_place.ttt
```

The scene should contain a RoboMaster EP model with the CoppeliaSim RoboMaster
plugin available. The script expects this object path:

```text
/RoboMaster
```

The script also expects this API to be available:

```lua
local simRobomaster = require('simRobomaster')
```

## Main Parameters

The most important tuning values are near the top of the Lua script:

```lua
local cubeX = 0.120
local cubeY = 0.000

local homeServo0 = -0.08
local homeServo1 = 0.00

local aboveServo1 = 0.65
local lowerServo0 = 0.58

local liftServo0 = 0.12
local liftServo1 = 0.55
```

Tuning guide:

- If the gripper reaches behind or in front of the cube, adjust `cubeX`.
- If the gripper is too high at the pick point, increase `lowerServo0`.
- If the gripper hits the ground or pushes the cube away, decrease
  `lowerServo0`.
- If the arm swings too fast, reduce `armSpeed`.
- If the turn is too aggressive, reduce `turnSpeed`.

## Run Procedure

1. Open the RoboMaster EP scene in CoppeliaSim.
2. Stop simulation before editing scripts.
3. Select the scene script from the scene hierarchy.
4. Paste in the content of:

```text
scripts/coppeliasim_arm_only_pick_place.lua
```

5. Save the scene.
6. Press Play.
7. Watch the status messages in the CoppeliaSim log. Each step logs its index
   and action name.

## Current Risks

- Physical friction grasp may be unstable if the cube is too light, the gripper
  contact area is too small, or the gripper closes too fast.
- The place marker is only a visual marker. The actual release location depends
  on the robot turn and arm lowering position.
- The current script uses timed waits. It does not yet confirm that servos have
  reached their target positions before moving to the next step.

## Recommended Next Steps

1. Save one successful run video or screen recording for the experiment record.
2. Record a small table of `cubeX`, `aboveServo1`, `lowerServo0`, and observed
   result.
3. If real-physics grasp stays unstable, add a separate demo-only attach/detach
   script and clearly label it as presentation assistance.
4. Keep the Gazebo ROS 2 route as the long-term implementation route for
   controller and report completeness.

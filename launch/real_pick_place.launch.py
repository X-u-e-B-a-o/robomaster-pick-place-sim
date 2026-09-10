from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                "python3",
                "/home/nvidia/colcon_ws/src/robomaster_pick_place_sim/real/real_pick_place_ros2_node.py",
            ],
            output="screen",
        )
    ])

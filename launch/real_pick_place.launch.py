from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动真机抓取节点 (板子上已连机器人热点时使用)。"""
    return LaunchDescription([
        Node(
            package="robomaster_pick_place_sim",
            executable="real_pick_place",
            name="real_pick_place_node",
            output="screen",
        )
    ])

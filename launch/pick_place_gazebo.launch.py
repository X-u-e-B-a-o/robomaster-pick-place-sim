from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('robomaster_pick_place_sim')
    urdf = os.path.join(pkg, 'urdf', 'robomaster_ep_gazebo.urdf')
    world = os.path.join(pkg, 'worlds', 'pick_place.sdf')
    robot_description = {'robot_description': Command(['xacro ', urdf])}

    return LaunchDescription([
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', '-v', '1', world, '--force-version', '6'], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[robot_description], output='screen'),
        TimerAction(period=3.0, actions=[
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', 'robomaster_ep_core', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0'],
                 output='screen')
        ]),
        TimerAction(period=6.0, actions=[
            ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'], output='screen'),
        ]),
    ])

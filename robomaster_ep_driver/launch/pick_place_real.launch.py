"""RoboMaster EP 真机定点抓取 launch.

与仿真 (pick_place_moveit.launch.py) 的差异仅是设备/通信层:
  - 不启动 Gazebo / 不 spawn 机器人 / 不启动 controller_manager spawner,
    由 ep_arm_driver 伪装 ros2_control 接口 (joint_states +
    FollowJointTrajectory + gripper commands + list_controllers);
  - use_sim_time 默认 false;
  - move_group 提高 allowed_execution_duration_scaling (真机 SDK 执行
    慢于规划时长, 默认 2.0 会被 trajectory_execution 判超时);
  - 任务节点原样复用 pick_place_moveit, 仅换参数文件 (点位 A/B 一前一后、
    低速、真机回零角)。

用法 (在 Jetson 上, EP 热点直连后):
  ros2 launch robomaster_ep_driver pick_place_real.launch.py
  # 干跑 (不控真机, 仅打印映射路径与状态):
  ros2 launch robomaster_ep_driver pick_place_real.launch.py dry_run:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('robomaster_pick_place_sim')
    moveit_pkg = get_package_share_directory('robomaster_ep_moveit_config')
    driver_pkg = get_package_share_directory('robomaster_ep_driver')
    urdf = os.path.join(pkg, 'urdf', 'robomaster_ep_gazebo.urdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    dry_run = LaunchConfiguration('dry_run', default='false')
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(driver_pkg, 'config',
                             'pick_place_params_real.yaml'))
    driver_params = LaunchConfiguration(
        'driver_params',
        default=os.path.join(driver_pkg, 'config', 'ep_driver_params.yaml'))
    exec_scaling = LaunchConfiguration('execution_duration_scaling',
                                       default='20.0')

    # 1. 机器人状态发布 (TF + robot_description, 与仿真同一 URDF)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(
                         Command(['xacro ', urdf]), value_type=str),
                     'use_sim_time': use_sim_time}],
        output='screen')

    # 2. EP 真机驱动 (伪装 ros2_control 接口, 经官方 SDK 控制机械臂)
    driver = Node(
        package='robomaster_ep_driver',
        executable='ep_arm_driver',
        parameters=[{'use_sim_time': use_sim_time,
                     'dry_run': dry_run,
                     'params_file': driver_params}],
        output='screen')

    # 3. MoveIt move_group (规划与轨迹执行, 与仿真同一套配置)
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'move_group.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'execution_duration_scaling': exec_scaling}.items())

    # 4. 定点抓取任务节点 (与仿真同一可执行, 仅换参数文件)
    pick_place = TimerAction(period=6.0, actions=[
        Node(package='robomaster_pick_place_sim',
             executable='pick_place_moveit',
             parameters=[{'use_sim_time': use_sim_time,
                          'params_file': params_file}],
             output='screen')])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params_file,
                              description='抓取任务参数文件 (真机默认 '
                                          'pick_place_params_real.yaml)'),
        DeclareLaunchArgument('driver_params', default_value=driver_params,
                              description='EP 驱动参数文件 (含标定锚点)'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true=不控真机, 仅打印映射路径'),
        DeclareLaunchArgument('execution_duration_scaling',
                              default_value='20.0',
                              description='轨迹执行时长上限倍率 (真机慢)'),
        rsp,
        driver,
        move_group,
        pick_place,
    ])

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(pkg_share, *path):
    with open(os.path.join(pkg_share, *path), 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_config_share = get_package_share_directory('robomaster_ep_moveit_config')
    sim_share = get_package_share_directory('robomaster_pick_place_sim')
    urdf_path = os.path.join(sim_share, 'urdf', 'robomaster_ep_gazebo.urdf')

    # URDF 里含 $(find ...) 和 ros2_control/gazebo 标签, 必须经 xacro 展开
    robot_description = {'robot_description': ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str)}
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            open(os.path.join(moveit_config_share, 'config', 'robomaster_ep.srdf')).read(),
            value_type=str)
    }

    kinematics = load_yaml(moveit_config_share, 'config', 'kinematics.yaml')
    joint_limits = load_yaml(moveit_config_share, 'config', 'joint_limits.yaml')
    moveit_controllers = load_yaml(moveit_config_share, 'config', 'moveit_controllers.yaml')
    ompl = load_yaml(moveit_config_share, 'config', 'ompl_planning.yaml')

    # 参数结构与官方 moveit_configs_utils.MoveItConfigsBuilder.to_dict() 一致:
    # - planning_pipelines 是管线名字列表 (move_group.cpp 读 vector<string>)
    # - 每条管线的配置 (planning_plugin/adapters/planner_configs) 作为顶层参数 ~<pipeline>.*
    # - moveit_controllers.yaml 内容顶层合并 (moveit_controller_manager +
    #   moveit_simple_controller_manager 直接落在 move_group 节点命名空间)
    # - robot_description_planning 只含关节限位
    planning_pipelines = {'planning_pipelines': ['ompl'],
                          'default_planning_pipeline': 'ompl',
                          'ompl': ompl}

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # 轨迹执行时长上限倍率 (MoveIt 默认 2.0): 真机 SDK 逐点执行比规划时长
    # 慢得多, 需要调大, 否则 trajectory_execution 会因超时提前中断
    exec_scaling = LaunchConfiguration('execution_duration_scaling', default='2.0')

    move_group_params = [
        robot_description,
        robot_description_semantic,
        {'robot_description_kinematics': kinematics},
        {'robot_description_planning': joint_limits},
        moveit_controllers,
        planning_pipelines,
        {
            'moveit_manage_controllers': True,
            'use_sim_time': use_sim_time,
            'allow_trajectory_execution': True,
            'monitor_dynamics': False,
            'publish_robot_description_semantic': True,
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
            'trajectory_execution': {
                'allowed_execution_duration_scaling': exec_scaling,
            },
        },
    ]

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_params,
    )

    return LaunchDescription([move_group_node])

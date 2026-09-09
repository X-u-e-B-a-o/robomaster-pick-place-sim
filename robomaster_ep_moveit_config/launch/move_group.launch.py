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

    # OMPL 规划管线: ompl_planning.yaml 与管线框架参数合并
    ompl.update({
        'move_group': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': (
                'default_planner_request_adapters/AddTimeParameterization '
                'default_planner_request_adapters/ResolveConstraintFrames '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'response_adapters': (
                'default_planning_response_adapters/AddTimeParameterization '
                'default_planning_response_adapters/ValidateSolution '
                'default_planning_response_adapters/DisplayMotionPath'
            ),
            'start_state_max_bounds_error': 0.1,
        }
    })
    planning_pipelines = {'ompl': ompl}

    # robot_description_planning = 关节限位 + 控制器配置
    robot_description_planning = {}
    robot_description_planning.update(joint_limits)
    robot_description_planning.update(moveit_controllers)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    move_group_params = [
        robot_description,
        robot_description_semantic,
        {'robot_description_kinematics': kinematics},
        {'robot_description_planning': robot_description_planning},
        {'trajectory_execution': moveit_controllers},
        {'planning_pipelines': planning_pipelines},
        {'default_planning_pipeline': 'ompl'},
        {
            'use_sim_time': use_sim_time,
            'publish_robot_description_semantic': True,
            'allow_trajectory_execution': True,
            'capabilities': (
                'move_group/MoveGroupExecuteService '
                'move_group/MoveGroupCartesianPathService '
                'move_group/MoveGroupMoveService '
                'move_group/MoveGroupKinematicsService'
            ),
            'monitor_dynamics': False,
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
        },
    ]

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_params,
    )

    return LaunchDescription([move_group_node])

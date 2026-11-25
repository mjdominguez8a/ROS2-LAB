"""MoveIt for the SO-100, on top of a running arm.launch.py (bonus task).

    ros2 launch so100_bringup arm.launch.py rviz:=false        # terminal 1 (any backend)
    ros2 launch so100_bringup moveit.launch.py                 # terminal 2: move_group + RViz

move_group plans for the `manipulator` group (tip: tcp_link) and executes through the same
joint_trajectory_controller / gripper_controller that the labs use, so it works with
backend:=mock, gazebo and real. Pass use_sim_time:=true with backend:=gazebo.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

PKG = 'so100_bringup'


def _setup(context):
    share = get_package_share_directory(PKG)
    moveit_share = get_package_share_directory('so_arm100_moveit_config')
    use_sim_time = context.launch_configurations['use_sim_time'] == 'true'
    rviz = context.launch_configurations['rviz'] == 'true'

    config = (
        MoveItConfigsBuilder('so100', package_name='so_arm100_moveit_config')
        # move_group only needs the kinematic model; ros2_control is already running.
        .robot_description(file_path=os.path.join(share, 'urdf', 'so100.urdf.xacro'),
                           mappings={'backend': 'mock'})
        .robot_description_semantic(file_path=os.path.join(share, 'config', 'so100.srdf'))
        .robot_description_kinematics(
            file_path=os.path.join(share, 'config', 'moveit_kinematics.yaml'))
        .joint_limits(file_path=os.path.join(moveit_share, 'config', 'joint_limits.yaml'))
        .trajectory_execution(
            file_path=os.path.join(moveit_share, 'config', 'trajectory_execution.yaml'))
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    common = [config.to_dict(), {'use_sim_time': use_sim_time}]
    actions = [Node(package='moveit_ros_move_group', executable='move_group',
                    output='screen', parameters=common)]
    if rviz:
        actions.append(Node(
            package='rviz2', executable='rviz2', output='log',
            arguments=['-d', os.path.join(moveit_share, 'rviz', 'move_group.rviz')],
            parameters=[config.robot_description, config.robot_description_semantic,
                        config.robot_description_kinematics,
                        config.planning_pipelines, config.joint_limits,
                        {'use_sim_time': use_sim_time}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_setup),
    ])

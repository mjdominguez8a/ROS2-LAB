"""Start the SO-100 arm: same controllers and topics on every backend.

    ros2 launch so100_bringup arm.launch.py                      # mock (RViz only)
    ros2 launch so100_bringup arm.launch.py backend:=gazebo      # physics simulation
    ros2 launch so100_bringup arm.launch.py backend:=real robot:=arm3   # real robot

    controller:=trajectory (default) -> joint_trajectory_controller is active
    controller:=position             -> forward_position_controller is active (sim only)
    sag:=true                        -> mock arm sags under gravity like the real one (Task 6)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

PKG = 'so100_bringup'


def _setup(context):
    share = get_package_share_directory(PKG)
    backend = context.launch_configurations['backend']
    controller = context.launch_configurations['controller']
    rviz = context.launch_configurations['rviz'] == 'true'
    gazebo_gui = context.launch_configurations['gazebo_gui'] == 'true'
    robot = context.launch_configurations['robot']
    usb_port = context.launch_configurations['usb_port']
    world = context.launch_configurations['world']
    sag = context.launch_configurations['sag'] == 'true'

    if backend not in ('mock', 'gazebo', 'real'):
        return [LogInfo(msg=f"Unknown backend '{backend}'. Use mock, gazebo or real."), Shutdown()]
    if controller not in ('trajectory', 'position'):
        return [LogInfo(msg=f"Unknown controller '{controller}'. Use trajectory or position."),
                Shutdown()]
    if backend == 'real' and controller == 'position':
        return [LogInfo(msg='controller:=position is disabled on the real robot (it jumps '
                            'straight to targets). Use controller:=trajectory.'), Shutdown()]

    if backend == 'real' and not os.path.exists(usb_port):
        return [LogInfo(msg=f'Robot not found at {usb_port}. Is the USB cable plugged in and the '
                            'power on? Did you run scripts/connect_robot.sh? '
                            '(see docs/REAL_ROBOT.md)'), Shutdown()]

    if sag and backend != 'mock':
        return [LogInfo(msg='sag:=true only works with backend:=mock (the real robot sags by '
                            'itself).'), Shutdown()]

    calibration = os.path.join(share, 'config', 'robots', f'{robot}.yaml')
    if not os.path.exists(calibration):
        return [LogInfo(msg=f'No calibration file {calibration}. Run: '
                            f'ros2 run so100_bringup calibrate --robot-id {robot}'), Shutdown()]

    controllers_file = os.path.join(share, 'config', 'controllers.yaml')
    sim_time = backend == 'gazebo'
    robot_description = ParameterValue(Command([
        'xacro ', os.path.join(share, 'urdf', 'so100.urdf.xacro'),
        f' backend:={backend} usb_port:={usb_port}',
        f' calibration_file:={calibration} controllers_file:={controllers_file}',
        # In Gazebo the arm stands on the table in table.sdf.
        ' z:=0.75' if backend == 'gazebo' else '',
    ]), value_type=str)

    actions = [Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='both',
        parameters=[{'robot_description': robot_description, 'use_sim_time': sim_time}])]

    if backend == 'gazebo':
        gz_args = f'-r -v 1 {world}' if gazebo_gui else f'-s -r -v 1 {world}'
        actions += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
                launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'so100']),
            Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        ]
    else:
        actions.append(Node(
            package='controller_manager', executable='ros2_control_node', output='both',
            parameters=[controllers_file] + (
                [os.path.join(share, 'config', 'controllers_real.yaml')] if backend == 'real' else []),
            remappings=[('~/robot_description', '/robot_description')]))

    active = ('joint_trajectory_controller' if controller == 'trajectory'
              else 'forward_position_controller')
    spawn = ['gripper_controller', active]
    # sag:=true: the broadcaster publishes the mock's perfect positions elsewhere, and
    # so100_labs' servo_sag_sim publishes sagged ones on /joint_states (see so100_labs/sag_sim.py).
    # Started by name only: so100_labs depends on this package, not the other way round.
    jsb_args = (['--controller-ros-args', '-r joint_states:=/so100/commanded_joint_states']
                if sag else [])
    actions.append(Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'] + jsb_args,
        parameters=[{'use_sim_time': sim_time}]))
    actions.append(Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=spawn + ['--controller-manager-timeout', '60'],
        parameters=[{'use_sim_time': sim_time}]))
    if sag:
        actions.append(Node(package='so100_labs', executable='servo_sag_sim', output='screen'))

    # The other arm controller is loaded but inactive, so students can switch with
    #   ros2 control switch_controllers --activate X --deactivate Y
    if backend != 'real':
        inactive = ('forward_position_controller' if controller == 'trajectory'
                    else 'joint_trajectory_controller')
        actions.append(Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=[inactive, '--inactive', '--controller-manager-timeout', '60'],
            parameters=[{'use_sim_time': sim_time}]))

    if backend == 'real':
        actions.append(Node(
            package=PKG, executable='safety_monitor', output='screen',
            parameters=[os.path.join(share, 'config', 'safety_limits.yaml')]))

    if rviz:
        actions.append(Node(
            package='rviz2', executable='rviz2', output='log',
            arguments=['-d', os.path.join(share, 'rviz', 'so100.rviz')],
            parameters=[{'use_sim_time': sim_time}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory(PKG)
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='mock',
                              description='mock | gazebo | real'),
        DeclareLaunchArgument('controller', default_value='trajectory',
                              description='trajectory | position (position: sim only)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='false',
                              description='Show the Gazebo window (slow without a GPU)'),
        DeclareLaunchArgument('sag', default_value='false',
                              description='mock only: simulate gravity sag of the servos'),
        DeclareLaunchArgument('robot', default_value='default',
                              description='Calibration file name in config/robots/'),
        DeclareLaunchArgument('usb_port', default_value='/dev/so100'),
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds', 'table.sdf')),
        OpaqueFunction(function=_setup),
    ])

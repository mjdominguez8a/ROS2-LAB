"""Arm: a small helper that hides the ROS plumbing for the SO-100 labs.

PROVIDED - you do not need to change this file, but reading it is a good way to
learn how ROS 2 topics, actions and TF are used.

The same code works on every backend (mock, gazebo, real) because they all expose
the same controllers:

    /joint_states                                          sensor_msgs/JointState
    /forward_position_controller/commands                  std_msgs/Float64MultiArray
    /joint_trajectory_controller/follow_joint_trajectory   control_msgs/FollowJointTrajectory
    /joint_trajectory_controller/joint_trajectory          trajectory_msgs/JointTrajectory
    /gripper_controller/gripper_cmd                        control_msgs/ParallelGripperCommand
    TF  base_link -> tcp_link

Safety: every command is checked against the joint limits and MAX_VELOCITY before
it is sent. On the real robot, run with `--ros-args -p speed_scale:=0.3` (the
`scripts/run_on_robot.sh` helper does this) to slow every trajectory down.
"""
import math
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from so100_labs.robot_model import (ARM_JOINTS, GRIPPER_JOINT, LOWER, MAX_VELOCITY,
                                    UPPER)


# stream_position(): the largest allowed distance [rad] between a streamed command and the
# measured position. A PID correction is a few degrees; anything bigger is a bug.
MAX_STREAM_STEP = 0.25


class SafetyError(ValueError):
    """Raised when a command would violate joint or velocity limits."""


def _duration(seconds: float) -> Duration:
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int((seconds - sec) * 1e9))


class Arm(Node):
    def __init__(self, name: str = 'so100_lab'):
        super().__init__(name)
        self.speed_scale = float(self.declare_parameter('speed_scale', 1.0).value)
        if not 0.05 <= self.speed_scale <= 1.0:
            raise SafetyError('speed_scale must be between 0.05 and 1.0')
        self._joint_state = None
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self._position_pub = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/so100_labs/markers', 10)
        self._stream_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self._last_stream = None
        self._trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self._gripper_client = ActionClient(
            self, ParallelGripperCommand, '/gripper_controller/gripper_cmd')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    # ------------------------------------------------------------------ state
    def _on_joint_state(self, msg: JointState):
        self._joint_state = msg

    def wait_until_ready(self, timeout: float = 30.0):
        """Block until joint states arrive (i.e. the arm is running)."""
        deadline = time.monotonic() + timeout
        while self._joint_state is None:
            if time.monotonic() > deadline:
                raise TimeoutError('No /joint_states received. Is arm.launch.py running?')
            rclpy.spin_once(self, timeout_sec=0.1)

    def joint_positions(self) -> list:
        """Current arm joint positions [rad], in ARM_JOINTS order."""
        self.wait_until_ready()
        rclpy.spin_once(self, timeout_sec=0.05)
        msg = self._joint_state
        return [msg.position[msg.name.index(j)] for j in ARM_JOINTS]

    def gripper_position(self) -> float:
        self.wait_until_ready()
        msg = self._joint_state
        return msg.position[msg.name.index(GRIPPER_JOINT)]

    def tcp_position(self, timeout: float = 2.0) -> np.ndarray:
        """Tool centre point position in base_link, measured through TF."""
        deadline = time.monotonic() + timeout
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                t = self._tf_buffer.lookup_transform('base_link', 'tcp_link', Time())
                v = t.transform.translation
                return np.array([v.x, v.y, v.z])
            except Exception:  # noqa: BLE001 - TF not available yet
                if time.monotonic() > deadline:
                    raise

    def sleep(self, seconds: float):
        """Wait while still processing ROS messages."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    # ------------------------------------------------------------ safety checks
    @staticmethod
    def check_limits(q):
        if len(q) != len(ARM_JOINTS):
            raise SafetyError(f'Expected {len(ARM_JOINTS)} joint values, got {len(q)}')
        for name, value, lo, hi in zip(ARM_JOINTS, q, LOWER, UPPER):
            if not math.isfinite(value) or value < lo - 1e-6 or value > hi + 1e-6:
                raise SafetyError(f'{name} = {value:.3f} rad is outside [{lo}, {hi}]')

    def check_trajectory(self, times, positions):
        start = self.joint_positions()
        if not times or len(times) != len(positions):
            raise SafetyError('times and positions must be non-empty and the same length')
        prev_t, prev_q = 0.0, start
        for t, q in zip(times, positions):
            self.check_limits(q)
            dt = t - prev_t
            if dt <= 0:
                raise SafetyError('times must be strictly increasing and > 0')
            speed = max(abs(a - b) for a, b in zip(q, prev_q)) / dt
            if speed > MAX_VELOCITY * 1.05:
                raise SafetyError(
                    f'Trajectory too fast: {speed:.2f} rad/s at t={t:.2f}s '
                    f'(limit {MAX_VELOCITY} rad/s). Use longer durations.')
            prev_t, prev_q = t, q

    # ---------------------------------------------------------------- commands
    def send_joint_positions(self, q):
        """Task 1: jump to `q` via forward_position_controller (simulation only)."""
        self.check_limits(q)
        self._position_pub.publish(Float64MultiArray(data=[float(v) for v in q]))

    def follow_trajectory(self, times, positions, velocities=None, timeout_margin=5.0) -> bool:
        """Execute a joint trajectory and block until it finishes.

        times      : list of seconds from start (strictly increasing)
        positions  : list of 5-element joint vectors (ARM_JOINTS order)
        velocities : optional list of 5-element joint velocity vectors
        """
        self.check_trajectory(times, positions)
        scale = self.speed_scale
        traj = JointTrajectory(joint_names=ARM_JOINTS)
        for i, (t, q) in enumerate(zip(times, positions)):
            point = JointTrajectoryPoint(positions=[float(v) for v in q],
                                         time_from_start=_duration(t / scale))
            if velocities is not None:
                point.velocities = [float(v) * scale for v in velocities[i]]
            traj.points.append(point)
        if not self._trajectory_client.wait_for_server(timeout_sec=10.0):
            raise TimeoutError('joint_trajectory_controller is not running '
                               '(launch with controller:=trajectory)')
        goal = FollowJointTrajectory.Goal(trajectory=traj)
        send = self._trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error('Trajectory rejected by the controller')
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result,
                                         timeout_sec=times[-1] / scale + timeout_margin)
        if not result.done():
            self.get_logger().error('Trajectory did not finish in time')
            return False
        code = result.result().result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'Trajectory failed: {result.result().result.error_string}')
            return False
        return True

    def move_to(self, q, duration=None) -> bool:
        """Move to joint vector `q` in a straight joint-space line, starting and ending at rest.

        The final point has zero velocity, so the controller interpolates a cubic that eases in
        and out (peak speed 1.5x the average) instead of a line that starts and stops abruptly.
        """
        if duration is None:
            delta = max(abs(a - b) for a, b in zip(q, self.joint_positions()))
            duration = max(1.0, 2.0 * delta / MAX_VELOCITY)
        return self.follow_trajectory([duration], [q], velocities=[[0.0] * len(q)])

    def stream_position(self, q, dt: float):
        """Task 6: command `q` to be reached in `dt` seconds, without waiting.

        Call it in a loop (e.g. 20 times per second) to steer the arm continuously. It publishes a
        one-point trajectory on joint_trajectory_controller's topic, which replaces the previous
        one, so it works on every backend, including the real robot.
        Raises SafetyError if `q` is outside the limits, more than MAX_STREAM_STEP away from the
        measured position, or too far from the previous command for `dt`.
        """
        self.check_limits(q)
        if dt <= 0:
            raise SafetyError('dt must be > 0')
        measured = self.joint_positions()
        far = max(abs(a - b) for a, b in zip(q, measured))
        if far > MAX_STREAM_STEP:
            raise SafetyError(f'Streamed command is {far:.2f} rad from the measured position '
                              f'(limit {MAX_STREAM_STEP}). Use move_to() for big moves.')
        prev, now = self._last_stream, time.monotonic()
        if prev is not None and now - prev[0] < 1.0:
            jump = max(abs(a - b) for a, b in zip(q, prev[1]))
            if jump > MAX_VELOCITY * max(dt, now - prev[0]) * 1.05:
                raise SafetyError(f'Streamed command jumps {jump:.3f} rad in {dt:.3f} s '
                                  f'(limit {MAX_VELOCITY} rad/s)')
        self._last_stream = (now, [float(v) for v in q])
        point = JointTrajectoryPoint(positions=[float(v) for v in q],
                                     time_from_start=_duration(dt))
        self._stream_pub.publish(JointTrajectory(joint_names=ARM_JOINTS, points=[point]))

    def gripper(self, position: float, timeout: float = 5.0) -> bool:
        """Move the gripper joint to `position` [rad] (see GRIPPER_OPEN / GRIPPER_CLOSED)."""
        if not self._gripper_client.wait_for_server(timeout_sec=10.0):
            raise TimeoutError('gripper_controller is not running')
        goal = ParallelGripperCommand.Goal()
        goal.command = JointState(name=[GRIPPER_JOINT], position=[float(position)])
        send = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        if not send.result().accepted:
            return False
        result = send.result().get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=timeout)
        return result.done()

    # ----------------------------------------------------------- visualisation
    def show_points(self, points, ns='targets', color=(0.1, 0.8, 0.2)):
        """Draw points (in base_link) as small spheres in RViz."""
        marker = Marker(type=Marker.SPHERE_LIST, action=Marker.ADD, ns=ns, id=0)
        marker.header.frame_id = 'base_link'
        marker.scale.x = marker.scale.y = marker.scale.z = 0.01
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, 1.0)
        marker.pose.orientation.w = 1.0
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
        self._marker_pub.publish(MarkerArray(markers=[marker]))


def run(main_function):
    """Initialise ROS, create an Arm, run `main_function(arm)`, shut down cleanly."""
    rclpy.init()
    arm = Arm()
    try:
        arm.wait_until_ready()
        ok = main_function(arm)
    except KeyboardInterrupt:
        ok = False
    finally:
        arm.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if ok in (None, True) else 1)

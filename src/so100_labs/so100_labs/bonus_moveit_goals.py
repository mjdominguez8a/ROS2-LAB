"""Bonus - MoveIt goals with position and orientation constraints.

PROVIDED example (no TODOs). Plans and executes three goals for the SAME target point:

  1. position only                       -> MoveIt may pick any gripper orientation
  2. position + "pointing down", free spin about the approach axis
  3. position + gripper pointing SIDEWAYS (horizontal, perpendicular to the arm) -> FAILS,
     even with free spin. With 5 joints, the approach direction must lie in the vertical
     plane through the base and the target (shoulder_pan picks the plane; lift/elbow/wrist
     move inside it; wrist_roll only spins about the approach axis). That is why Task 4
     solves for xyz + pitch: exactly the 4 things the first 4 joints can control.

MoveIt reports SUCCESS when the *planned* trajectory has been executed. It does not check where
the gripper really ended up: on the real robot (or with `arm.launch.py sag:=true`) the TCP is
several millimetres from the target, because the servos sag under gravity. Task 6 fixes that.

Run:
    ros2 launch so100_bringup arm.launch.py rviz:=false        # terminal 1 (add sag:=true)
    ros2 launch so100_bringup moveit.launch.py                 # terminal 2 (RViz)
    ros2 run so100_labs bonus_moveit_goals                     # terminal 3
    ros2 run so100_labs bonus_moveit_goals --ros-args -p plan_only:=true   # plan, don't move

How an OrientationConstraint works:
  * `orientation` is the target orientation of `link_name`, expressed in header.frame_id.
  * absolute_{x,y,z}_axis_tolerance: allowed rotation error about the x, y, z axes of the
    HEADER frame (here base_link), with parameterization = ROTATION_VECTOR. ~pi = free.
    (Careful: these are NOT the link's own axes. Verified on MoveIt 2 Jazzy.)
  * So to allow "free spin about the approach direction", give pi tolerance to the
    base_link axis that is PARALLEL to the approach direction:
      pointing down (approach = base -z)  -> tolerances (0.1, 0.1, pi)
      pointing along base +x              -> tolerances (pi, 0.1, 0.1)
  * For the SO-100 tcp_link, the fingers point along -y. "Pointing down" means link -y = base -z,
    which is a +90 deg rotation about x: quaternion (x, y, z, w) = (0.7071, 0, 0, 0.7071).
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint)
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive

from so100_labs.arm import Arm
from so100_labs.robot_model import POSES

FRAME = 'base_link'
LINK = 'tcp_link'
TARGET = (0.0, -0.22, 0.06)  # metres in base_link, straight in front of the arm


def quaternion_about_axis(axis, angle):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    s = math.sin(angle / 2)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2)])


# Pointing down: rotate +90 deg about base x so that the link's -y axis points along base -z.
POINTING_DOWN = quaternion_about_axis((1, 0, 0), math.pi / 2)


def quaternion_from_matrix(m):
    """Quaternion (x, y, z, w) of a 3x3 rotation matrix."""
    w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    return np.array([x, y, z, w])


def approach_quaternion(direction):
    """An orientation whose approach axis (link -y) points along `direction` (base frame)."""
    y_axis = -np.asarray(direction, float) / np.linalg.norm(direction)
    helper = np.array([0.0, 0.0, 1.0]) if abs(y_axis[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x_axis = np.cross(y_axis, helper)
    x_axis /= np.linalg.norm(x_axis)
    z_axis = np.cross(x_axis, y_axis)
    return quaternion_from_matrix(np.column_stack([x_axis, y_axis, z_axis]))


# Horizontal along base +x: perpendicular to the arm's vertical plane (the y-z plane at pan = 0).
SIDEWAYS = approach_quaternion((1.0, 0.0, 0.0))


def position_constraint(xyz, radius=0.005):
    pc = PositionConstraint()
    pc.header.frame_id = FRAME
    pc.link_name = LINK
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = xyz
    pose.orientation.w = 1.0
    pc.constraint_region = BoundingVolume(
        primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[radius])],
        primitive_poses=[pose])
    pc.weight = 1.0
    return pc


def orientation_constraint(quat, tol_xyz):
    oc = OrientationConstraint()
    oc.header.frame_id = FRAME
    oc.link_name = LINK
    oc.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]),
                                w=float(quat[3]))
    oc.absolute_x_axis_tolerance, oc.absolute_y_axis_tolerance, \
        oc.absolute_z_axis_tolerance = tol_xyz
    oc.parameterization = OrientationConstraint.ROTATION_VECTOR
    oc.weight = 1.0
    return oc


GOALS = [
    ('1. position only',
     Constraints(position_constraints=[position_constraint(TARGET)])),
    ('2. position + pointing down (free spin about the approach axis)',
     Constraints(position_constraints=[position_constraint(TARGET)],
                 orientation_constraints=[orientation_constraint(
                     POINTING_DOWN, (0.1, 0.1, math.pi))])),   # free about base z = approach
    ('3. position + pointing sideways (free spin) - impossible with 5 joints',
     Constraints(position_constraints=[position_constraint(TARGET)],
                 orientation_constraints=[orientation_constraint(
                     SIDEWAYS, (math.pi, 0.1, 0.1))])),        # free about base x = approach
]


class MoveItGoals(Arm):
    def __init__(self):
        super().__init__('bonus_moveit_goals')
        self.plan_only = bool(self.declare_parameter('plan_only', False).value)
        self.move_group = ActionClient(self, MoveGroup, '/move_action')

    def send(self, constraints):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'manipulator'
        req.goal_constraints = [constraints]
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3 * self.speed_scale
        req.max_acceleration_scaling_factor = 0.3
        req.start_state.is_diff = True
        goal.planning_options.plan_only = self.plan_only
        send = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        if not send.result().accepted:
            return MoveItErrorCodes.FAILURE
        result = send.result().get_result_async()
        rclpy.spin_until_future_complete(self, result)
        return result.result().result.error_code.val

    def tilt_from_down_deg(self):
        """Angle between the gripper approach direction and straight down (measured with TF)."""
        t = self._tf_buffer.lookup_transform(FRAME, LINK, rclpy.time.Time()).transform.rotation
        x, y, z, w = t.x, t.y, t.z, t.w
        # second column of the rotation matrix = link y axis in base frame; approach = -y
        approach = -np.array([2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)])
        return math.degrees(math.acos(np.clip(-approach[2], -1.0, 1.0)))


def main():
    rclpy.init()
    node = MoveItGoals()
    try:
        node.wait_until_ready()
        if not node.move_group.wait_for_server(timeout_sec=15.0):
            node.get_logger().error('move_group not running: ros2 launch so100_bringup '
                                    'moveit.launch.py')
            raise SystemExit(1)
        node.show_points([TARGET])
        for name, constraints in GOALS:
            node.move_to(POSES['home'])
            code = node.send(constraints)
            if code == MoveItErrorCodes.SUCCESS:
                node.sleep(1.0)
                tcp = node.tcp_position()  # measured: TF is computed from /joint_states
                extra = '' if node.plan_only else (
                    f', TCP at {np.round(tcp, 3)}, '
                    f'{1000 * np.linalg.norm(tcp - np.array(TARGET)):.1f} mm from the target, '
                    f'gripper tilted {node.tilt_from_down_deg():.0f} deg from straight down')
                node.get_logger().info(f'{name}: SUCCESS{extra}')
            else:
                node.get_logger().warn(f'{name}: FAILED (MoveIt error code {code})')
        node.move_to(POSES['home'])
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

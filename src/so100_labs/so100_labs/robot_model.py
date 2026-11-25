"""SO-100 robot model: joint names, limits and the kinematic chain.

PROVIDED - you do not need to change this file.

All numbers are copied from the URDF (so_arm100_description/urdf/so_arm100_macro.xacro
and so100_bringup/urdf/so100.urdf.xacro). Every pose in the labs is expressed in
the `base_link` frame: z points up, the arm reaches towards -y at shoulder_pan = 0.
"""
import math

# The 5 arm joints, in the order the controllers expect them.
ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_flex_joint',
    'wrist_flex_joint',
    'wrist_roll_joint',
]
GRIPPER_JOINT = 'gripper_joint'

# Position limits [rad] from the URDF.
LOWER = [-2.0, -1.75, -1.575, -3.5, -1.0]
UPPER = [2.0, 1.75, 1.57, 0.2, 4.0]
# Velocity limit used by the labs [rad/s] (the servos can do more; we stay gentle).
MAX_VELOCITY = 1.0

GRIPPER_OPEN = 1.2
GRIPPER_CLOSED = 0.0

# Kinematic chain: one entry per arm joint, from base_link outward.
#   xyz, rpy : fixed transform from the parent link to the joint frame (URDF <origin>)
#   axis     : rotation axis in the joint frame (URDF <axis>)
CHAIN = [
    # name                  xyz                       rpy                  axis
    ('shoulder_pan_joint', (0.0, -0.0452, 0.0165), (1.57079, 0.0, 0.0), (0.0, -1.0, 0.0)),
    ('shoulder_lift_joint', (0.0, 0.1025, 0.0306), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
    ('elbow_flex_joint', (0.0, 0.11257, 0.028), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
    ('wrist_flex_joint', (0.0, 0.0052, 0.1349), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
    ('wrist_roll_joint', (0.0, -0.0601, 0.0), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
]
# Fixed transform from the last link (gripper_link) to the tool centre point (tcp_link).
TCP_XYZ = (-0.0085, -0.0980, 0.0)
# In gripper_link coordinates the fingers point along -y: this is the "approach" direction.
APPROACH_AXIS = (0.0, -1.0, 0.0)

# Named joint-space poses [pan, lift, elbow, wrist_flex, wrist_roll].
POSES = {
    'zero': [0.0, 0.0, 0.0, 0.0, 0.0],
    'home': [0.0, -1.0, 1.2, -0.5, 1.57],
    'ready': [0.0, -0.3, 0.6, -1.2, 1.57],
    'left': [0.8, -0.3, 0.6, -1.2, 1.57],
    'right': [-0.8, -0.3, 0.6, -1.2, 1.57],
}

TAU = 2 * math.pi

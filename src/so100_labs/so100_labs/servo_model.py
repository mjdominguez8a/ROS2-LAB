"""A simple model of the SO-100 servos holding the arm up against gravity.

PROVIDED - you do not need to change this file. Task 6 uses it to test your PID controller
without a robot, and `arm.launch.py sag:=true` uses it to make the mock arm sag like the real one.

Why the real arm never quite reaches its target
-----------------------------------------------
Each STS3215 servo holds its position with its own P-only controller: the motor torque is
roughly proportional to the position error,

    tau_motor = K * (q_cmd - q)

Gravity pulls on the links with a torque tau_g(q) that depends on the whole arm's pose. The joint
stops where the two cancel, tau_motor + tau_g = 0, i.e.

    q = q_cmd + tau_g(q) / K          <- a steady-state error ("sag") that never goes away

A P-only controller needs an error to produce torque, so the error can only be zero if there is no
load. This is exactly what the integral term of a PID fixes.

The model: gravity torques from the URDF link masses, one spring (stiffness K) per servo tuned so
the sag matches what we measured on the lab arms (1-3 degrees on the loaded joints with the arm
stretched out, 1-2 cm at the gripper), a first-order lag (the servo needs time to move) and the encoder
resolution (4096 steps per turn).
"""
import math

import numpy as np

from so100_labs.robot_model import CHAIN, LOWER, TCP_XYZ, UPPER

GRAVITY = 9.81
ENCODER_STEP = 2 * math.pi / 4096  # rad, resolution of the STS3215 position sensor

# Mass [kg] and centre of mass (in the link's own frame) of the link moved by each joint, from
# so_arm100_macro.xacro. The jaw (0.020 kg) is lumped into the gripper link.
LINKS = [
    (0.119226, (-9.07886e-05, 0.0590972, 0.031089)),       # shoulder_link   (shoulder_pan)
    (0.162409, (-1.72052e-05, 0.0701802, 0.00310545)),     # upper_arm_link  (shoulder_lift)
    (0.147968, (-0.00339604, 0.00137796, 0.0768007)),      # lower_arm_link  (elbow_flex)
    (0.0661321, (-0.00852653, -0.0352279, -2.34622e-05)),  # wrist_link      (wrist_flex)
    (0.1132303, (0.00552377, -0.0280167, 0.000483583)),    # gripper_link + jaw (wrist_roll)
]

# Servo stiffness [Nm/rad] per joint: how much torque the servo's P controller makes per radian of
# error. Fitted to the sag measured on arm3 (P coefficient 32, D 32) at the two Task 6 poses:
# shoulder_lift ~0.015 rad, elbow_flex ~0.045 rad, wrist_flex ~0.019 rad. The real elbow varies
# from run to run (backlash, friction); the model gives the average. Pan and roll carry no
# gravity load, so their value barely matters.
STIFFNESS = [10.0, 33.0, 11.0, 4.8, 10.0]
TIME_CONSTANT = 0.08  # s, first-order response of a servo to a new command


def _rot(axis, angle):
    k = np.asarray(axis, float)
    k = k / np.linalg.norm(k)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * kx + (1 - math.cos(angle)) * (kx @ kx)


def _rpy(roll, pitch, yaw):
    return _rot((0, 0, 1), yaw) @ _rot((0, 1, 0), pitch) @ _rot((1, 0, 0), roll)


def joint_frames(q):
    """For each joint: (origin, axis, R, p) in base_link, where (R, p) is the pose of the link the
    joint moves. Also returns the TCP position. Same conventions as Task 3."""
    r, p = np.eye(3), np.zeros(3)
    frames = []
    for (_, xyz, rpy, axis), qi in zip(CHAIN, q):
        p = p + r @ np.asarray(xyz, float)
        r = r @ _rpy(*rpy)
        origin, world_axis = p.copy(), r @ np.asarray(axis, float)
        r = r @ _rot(axis, qi)
        frames.append((origin, world_axis, r.copy(), p.copy()))
    return frames, p + r @ np.asarray(TCP_XYZ, float)


def tcp_position(q):
    """TCP position in base_link (same result as Task 3's tcp_position)."""
    return joint_frames(q)[1]


def gravity_torques(q):
    """Torque [Nm] that gravity applies about each joint axis at pose q."""
    frames, _ = joint_frames(q)
    coms = [p + r @ np.asarray(com) for (_, _, r, p), (_, com) in zip(frames, LINKS)]
    weights = [np.array([0.0, 0.0, -mass * GRAVITY]) for mass, _ in LINKS]
    tau = []
    for j, (origin, axis, _, _) in enumerate(frames):
        moment = sum(np.cross(c - origin, w) for c, w in zip(coms[j:], weights[j:]))
        tau.append(float(np.dot(moment, axis)))
    return np.array(tau)


def steady_state(q_cmd, iterations=20):
    """Where the arm settles for a constant command q_cmd: q = q_cmd + tau_g(q) / K."""
    q_cmd = np.asarray(q_cmd, float)
    q = q_cmd.copy()
    for _ in range(iterations):
        q = q_cmd + gravity_torques(q) / np.asarray(STIFFNESS)
    return np.clip(q, LOWER, UPPER)


class ServoArmSim:
    """The 5 arm servos, stepped in time.

        sim = ServoArmSim(q0)
        q_measured = sim.step(q_command, dt)   # what /joint_states would report after dt seconds

    `delay_steps` delays the command by that many steps (like the time a message takes to reach
    the robot).
    """

    def __init__(self, q0, stiffness=STIFFNESS, time_constant=TIME_CONSTANT, delay_steps=1,
                 quantize=True):
        self.q = np.asarray(q0, float).copy()
        self.stiffness = np.asarray(stiffness, float)
        self.time_constant = time_constant
        self.quantize = quantize
        self._queue = [self.q.copy()] * delay_steps

    def measured(self):
        if not self.quantize:
            return self.q.copy()
        return np.round(self.q / ENCODER_STEP) * ENCODER_STEP

    def step(self, q_cmd, dt):
        self._queue.append(np.asarray(q_cmd, float).copy())
        cmd = self._queue.pop(0)
        target = cmd + gravity_torques(self.q) / self.stiffness
        self.q = self.q + (target - self.q) * (1.0 - math.exp(-dt / self.time_constant))
        self.q = np.clip(self.q, LOWER, UPPER)
        return self.measured()

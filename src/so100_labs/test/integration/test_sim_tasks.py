"""Run each task's node against the simulated (mock) arm and check the result.

Skipped by default. Run them after `colcon build` and `source install/setup.bash`:

    SO100_SIM_TESTS=1 python3 -m pytest src/so100_labs/test/integration -k task2 -v

Each test starts `arm.launch.py backend:=mock rviz:=false` (Task 6 adds `sag:=true`), runs
`ros2 run so100_labs <task>` and checks that it exits successfully and that the
arm ends in the expected pose.
"""
import os
import random
import signal
import subprocess
import time

import pytest

pytestmark = [
    pytest.mark.sim,
    pytest.mark.skipif(os.environ.get('SO100_SIM_TESTS') != '1',
                       reason='set SO100_SIM_TESTS=1 to run simulation tests'),
]

HOME = [0.0, -1.0, 1.2, -0.5, 1.57]
ARM_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_flex_joint',
              'wrist_flex_joint', 'wrist_roll_joint']


def _wait_for_controllers(expected, timeout=60):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        out = subprocess.run(['ros2', 'control', 'list_controllers'],
                             capture_output=True, text=True, timeout=20).stdout
        if all(any(line.split()[:1] == [c] and 'active' in line.split()
                   for line in out.splitlines()) for c in expected):
            return
        time.sleep(1.0)
    raise TimeoutError(f'controllers {expected} not active:\n{out}')


@pytest.fixture
def arm_sim(request, monkeypatch):
    param = getattr(request, 'param', 'trajectory')
    controller = 'trajectory' if param == 'sag' else param
    extra = ['sag:=true'] if param == 'sag' else []
    # Private DDS domain per test: stray nodes from earlier runs can't interfere.
    monkeypatch.setenv('ROS_DOMAIN_ID', str(random.randint(1, 100)))
    proc = subprocess.Popen(
        ['ros2', 'launch', 'so100_bringup', 'arm.launch.py', 'backend:=mock', 'rviz:=false',
         f'controller:={controller}'] + extra,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        active = ('joint_trajectory_controller' if controller == 'trajectory'
                  else 'forward_position_controller')
        _wait_for_controllers(['joint_state_broadcaster', 'gripper_controller', active])
        yield
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            pass
        try:  # make sure no child (controller manager, ...) survives
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def _run_task(executable, timeout):
    result = subprocess.run(['ros2', 'run', 'so100_labs', executable],
                            capture_output=True, text=True, timeout=timeout)
    print(result.stdout[-3000:], result.stderr[-3000:])
    return result.returncode


def _final_joint_positions():
    import rclpy
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node('sim_test_reader')
    msgs = []
    node.create_subscription(JointState, '/joint_states', msgs.append, 10)
    deadline = time.monotonic() + 10
    while not msgs and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    assert msgs, 'no /joint_states received'
    msg = msgs[-1]
    return [msg.position[msg.name.index(j)] for j in ARM_JOINTS]


def _assert_at_home(tolerance=0.02):
    q = _final_joint_positions()
    assert q == pytest.approx(HOME, abs=tolerance), f'arm should end at home, is at {q}'


@pytest.mark.parametrize('arm_sim', ['position'], indirect=True)
def test_task1_joint_commands(arm_sim):
    assert _run_task('task1_joint_commands', timeout=120) == 0
    _assert_at_home()


def test_task2_trajectories(arm_sim):
    assert _run_task('task2_trajectories', timeout=120) == 0
    _assert_at_home()


def test_task3_forward_kinematics(arm_sim):
    # exits non-zero if FK and TF disagree by more than 1 mm
    assert _run_task('task3_forward_kinematics', timeout=120) == 0
    _assert_at_home()


def test_task4_inverse_kinematics(arm_sim):
    assert _run_task('task4_inverse_kinematics', timeout=120) == 0
    _assert_at_home()


def test_task5_cartesian(arm_sim):
    assert _run_task('task5_cartesian', timeout=240) == 0
    _assert_at_home()


@pytest.mark.parametrize('arm_sim', ['sag'], indirect=True)
def test_task6_pid_control(arm_sim):
    # exits non-zero unless the PID brings the sagging arm within 3 mm of both targets
    assert _run_task('task6_pid_control', timeout=120) == 0
    _assert_at_home(tolerance=0.08)  # the final move home is open loop, so it sags again

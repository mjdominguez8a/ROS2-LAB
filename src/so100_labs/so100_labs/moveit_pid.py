"""task6_moveit_pid: your Task 6 PID as a finishing step after every MoveIt motion.

PROVIDED - needs your PID and corrected_command from task6_pid_control.py.

MoveIt plans and executes; when the trajectory ends, the servos sag and the gripper stops short.
This node watches joint_trajectory_controller: when a trajectory (from MoveIt, or anything else)
finishes, it takes the trajectory's last point as the target and holds it with your PID for
`duration` seconds. You see the arm climb onto the target, and the log shows the TCP error before
and after. As soon as a new trajectory starts, it stops, so it never fights MoveIt.

    ros2 launch so100_bringup arm.launch.py backend:=real robot:=arm3    # or: sag:=true
    ros2 launch so100_bringup moveit.launch.py
    ros2 run so100_labs task6_moveit_pid
Then drag the marker in RViz and click Plan & Execute. Switch the PID off/on while it runs:
    ros2 param set /moveit_pid enabled false
In RViz, add a MarkerArray display on /so100_labs/markers: the red dot is the target TCP.
"""
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from control_msgs.msg import JointTrajectoryControllerState
from rcl_interfaces.msg import SetParametersResult

from so100_labs.arm import Arm, SafetyError
from so100_labs.robot_model import ARM_JOINTS
from so100_labs.servo_model import tcp_position
from so100_labs.task6_pid_control import RATE, corrected_command, make_pids, tcp_error_mm

CONTROLLER = '/joint_trajectory_controller'


class MoveItPid(Arm):
    def __init__(self):
        super().__init__('moveit_pid')
        self.enabled = bool(self.declare_parameter('enabled', True).value)
        self.duration = float(self.declare_parameter('duration', 3.0).value)
        self.add_on_set_parameters_callback(self._on_params)
        self.executing = False
        self.reference = None   # controller setpoint, recorded while a trajectory runs
        self.finished = None    # target of the trajectory that just ended
        self.create_subscription(GoalStatusArray,
                                 f'{CONTROLLER}/follow_joint_trajectory/_action/status',
                                 self._on_status, 10)
        self.create_subscription(JointTrajectoryControllerState, f'{CONTROLLER}/controller_state',
                                 self._on_state, 10)

    def _on_params(self, params):
        for p in params:
            if p.name == 'enabled':
                self.enabled = bool(p.value)
                self.get_logger().info(f'PID {"ON" if self.enabled else "OFF"}')
            elif p.name == 'duration':
                self.duration = float(p.value)
        return SetParametersResult(successful=True)

    def _on_status(self, msg):
        executing = any(s.status == GoalStatus.STATUS_EXECUTING for s in msg.status_list)
        if self.executing and not executing and self.reference is not None:
            self.finished = list(self.reference)
        self.executing = executing

    def _on_state(self, msg):
        if self.executing and all(j in msg.joint_names for j in ARM_JOINTS):
            self.reference = [msg.reference.positions[msg.joint_names.index(j)]
                              for j in ARM_JOINTS]

    def report(self, label, target):
        mm = tcp_error_mm(target, self.joint_positions())
        self.get_logger().info(f'{label:18s} TCP off by {mm:5.1f} mm')

    def hold(self, target):
        pids = make_pids()
        period = 1.0 / RATE
        start = time.monotonic()
        next_tick, last = start, None
        while time.monotonic() - start < self.duration:
            if self.executing or not self.enabled:
                return False  # a new trajectory started (or PID switched off): stop at once
            now = time.monotonic()
            dt = period if last is None else now - last
            last = now
            cmd = corrected_command(target, self.joint_positions(), pids, dt)
            self.stream_position(cmd, period)
            next_tick += period
            self.sleep(max(0.0, next_tick - time.monotonic()))
        return True

    def run(self):
        self.get_logger().info('Waiting for trajectories (Plan & Execute in RViz). '
                               f'PID is {"ON" if self.enabled else "OFF"}.')
        while rclpy.ok():
            self.sleep(0.05)
            if self.finished is None or self.executing:
                continue
            target, self.finished = self.finished, None
            self.show_points([tcp_position(target)], ns='pid_target', color=(0.9, 0.1, 0.1))
            self.sleep(0.5)  # let the servos settle
            self.report('MoveIt done:', target)
            if not self.enabled:
                continue
            try:
                if self.hold(target):
                    self.report('after your PID:', target)
            except SafetyError as e:
                self.get_logger().error(f'PID stopped: {e}')
            except (NotImplementedError, TypeError) as e:
                self.get_logger().error(f'{e} - complete the TODOs in task6_pid_control.py first')
                return


def main():
    rclpy.init()
    node = MoveItPid()
    try:
        node.wait_until_ready()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

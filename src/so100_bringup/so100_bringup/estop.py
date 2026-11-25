"""Soft emergency stop: turns servo torque OFF on the real robot.

    ros2 run so100_bringup estop            # torque off (SUPPORT THE ARM - it will drop)
    ros2 run so100_bringup estop --resume   # torque back on

The power switch is always the real emergency stop.
"""
import sys

import rclpy
from controller_manager_msgs.srv import SetHardwareComponentState
from lifecycle_msgs.msg import State


def main():
    resume = '--resume' in sys.argv
    rclpy.init()
    node = rclpy.create_node('estop')
    client = node.create_client(SetHardwareComponentState,
                                '/controller_manager/set_hardware_component_state')
    if not client.wait_for_service(timeout_sec=3.0):
        print('controller_manager not running. Use the power switch.')
        sys.exit(1)
    target = (State(id=State.PRIMARY_STATE_ACTIVE, label='active') if resume
              else State(id=State.PRIMARY_STATE_INACTIVE, label='inactive'))
    future = client.call_async(SetHardwareComponentState.Request(name='SO100', target_state=target))
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    ok = future.result() is not None and future.result().ok
    print(('Torque ON (resumed)' if resume else 'Torque OFF') if ok else 'Request failed!')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

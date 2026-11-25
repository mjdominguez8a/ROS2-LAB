"""servo_sag_sim: make the mock arm sag under gravity like the real one.

PROVIDED - started by `ros2 launch so100_bringup arm.launch.py sag:=true` (mock backend only).

The mock hardware is perfect: it reports exactly what the controller commanded. With sag:=true,
joint_state_broadcaster publishes those perfect positions on /so100/commanded_joint_states
instead, and this node turns them into what real servos would measure (servo_model.ServoArmSim:
gravity sag, lag, encoder resolution) on /joint_states. RViz, TF, MoveIt and your code all see
the sagging arm; the controllers still think they are perfect, like on the real robot.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from so100_labs.robot_model import ARM_JOINTS
from so100_labs.servo_model import ServoArmSim

RATE = 50.0  # Hz, same as the controller manager


class SagSim(Node):
    def __init__(self):
        super().__init__('servo_sag_sim')
        self.sim = None
        self.commanded = None
        self.create_subscription(JointState, '/so100/commanded_joint_states', self.on_cmd, 10)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / RATE, self.tick)
        self.get_logger().info('Simulating gravity sag: /so100/commanded_joint_states -> '
                               '/joint_states')

    def on_cmd(self, msg):
        self.commanded = msg

    def tick(self):
        msg = self.commanded
        if msg is None or not all(j in msg.name for j in ARM_JOINTS):
            return
        q_cmd = [msg.position[msg.name.index(j)] for j in ARM_JOINTS]
        if self.sim is None:
            self.sim = ServoArmSim(q_cmd, delay_steps=0)
        q = self.sim.step(q_cmd, 1.0 / RATE)
        out = JointState(name=list(msg.name), position=list(msg.position),
                         velocity=list(msg.velocity), effort=list(msg.effort))
        out.header.stamp = self.get_clock().now().to_msg()
        for j, value in zip(ARM_JOINTS, q):
            out.position[out.name.index(j)] = float(value)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SagSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

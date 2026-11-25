"""Lab 2: drive the turtlesim turtle in a square using velocity commands."""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

TIMER_PERIOD = 0.1      # seconds between commands
FORWARD_SPEED = 1.0     # m/s
TURN_SPEED = 1.0        # rad/s
SIDE_TICKS = 20         # 20 ticks * 0.1 s * 1.0 m/s = 2 m per side
TURN_TICKS = 16         # 16 ticks * 0.1 s * 1.0 rad/s ~= 1.6 rad ~= 90 degrees


def compute_command(tick):
    """Return (linear_x, angular_z) for a given tick of the square pattern.

    Pure function with no ROS dependency, so it can be unit tested.
    """
    phase = tick % (SIDE_TICKS + TURN_TICKS)
    if phase < SIDE_TICKS:
        return FORWARD_SPEED, 0.0
    return 0.0, TURN_SPEED


class TurtleSquare(Node):
    def __init__(self):
        super().__init__('turtle_square')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.tick = 0
        self.timer = self.create_timer(TIMER_PERIOD, self.step)

    def step(self):
        linear, angular = compute_command(self.tick)
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.publisher.publish(cmd)
        self.tick += 1


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSquare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

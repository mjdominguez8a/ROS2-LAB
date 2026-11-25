"""Exercise: make the turtle drive in a circle.

TODO (student):
  1. Complete compute_circle_command() so the turtle moves forward AND turns
     at the same time. Hint: a circle needs both linear.x and angular.z.
  2. Rebuild with:  cb
  3. Run turtlesim in one terminal:   ros2 run turtlesim turtlesim_node
     and this node in another:        ros2 run lab_starter turtle_circle
  4. Check your work:  colcon test --packages-select lab_starter && colcon test-result --verbose
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def compute_circle_command(radius, speed):
    """Return (linear_x, angular_z) to drive a circle of `radius` metres at `speed` m/s.

    TODO: replace the line below. Hint: angular velocity = speed / radius.
    """
    return 0.0, 0.0


class TurtleCircle(Node):
    def __init__(self):
        super().__init__('turtle_circle')
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('speed', 1.0)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.step)

    def step(self):
        radius = self.get_parameter('radius').value
        speed = self.get_parameter('speed').value
        linear, angular = compute_circle_command(radius, speed)
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCircle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

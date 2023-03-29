import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class CreateControllerPub(Node):

    def __init__(self):
        super().__init__('create_controller_pub')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg}")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    create_controller_pub = CreateControllerPub()

    rclpy.spin(create_controller_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
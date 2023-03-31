import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import termios
import tty

from ps4_controller import PS4Controller

class CreateControllerPub(Node):

    def __init__(self):
        super().__init__('create_controller_pub')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist_msg = Twist()
        self.controller = PS4Controller(interface="/dev/input/js0", connecting_using_ds4drv=False)
        
    def publish_msg(self, drive, turn):
        self.twist_msg.linear.x = drive
        self.twist_msg.angular.z = turn
        self.publisher.publish(msg)
    

def main():
    rclpy.init()
    create_controller_pub = CreateControllerPub()
    rclpy.spin(create_controller_pub)

    minimal_publisher.destroy_node()
    rclpy.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
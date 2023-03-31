import threading
import random

import rclpy
from rclpy.node import Node

from .ps4_controller import PS4Controller
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import LightringLeds
from irobot_create_msgs.msg import LedColor

class CreateControllerPub(Node):
    def __init__(self, controller):
        super().__init__("create_controller_pub")
        self.twist_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.led_publisher = self.create_publisher(LightringLeds, "cmd_lightring", 10)
        
        self.twist_msg = Twist()
        self.led_msg = LightringLeds()

        self.controller = controller
        
    def start_up_controller(self):
        self.controller.register_cmd_vel_pub_cb(self.publish_twist_msg)
        self.controller.register_cmd_led_pub_cb(self.publish_led)
        self.ct = threading.Thread(target=self.controller.spin_controller)
        self.ct.start()
    
    def publish_twist_msg(self, drive, turn):
        self.twist_msg.linear.x = drive
        self.twist_msg.angular.z = turn
        self.twist_publisher.publish(self.twist_msg)
    
    def publish_led(self):
        rand_color_led = LedColor()
        rand_color_led.red = random.randint(0,255)
        rand_color_led.green = random.randint(0,255)
        rand_color_led.red = random.randint(0,255)

        for i in range(0,6):
            self.led_msg.leds[i] = rand_color_led
        
        self.led_publisher.publish(self.led_msg)


def main():
    rclpy.init()
    controller = PS4Controller(
        interface="/dev/input/js0", connecting_using_ds4drv=False
    )
    create_controller_pub = CreateControllerPub(controller)
    create_controller_pub.start_up_controller()
    rclpy.spin(create_controller_pub)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

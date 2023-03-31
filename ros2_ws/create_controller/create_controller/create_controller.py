import threading
import random

import rclpy
from rclpy.node import Node

from .ps4_controller import PS4Controller
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import LightringLeds
from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import HazardDetection

import math, time

class CreateControllerPub(Node):
    def __init__(self, controller):
        super().__init__("create_controller_pub")
        self.twist_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.led_publisher = self.create_publisher(LightringLeds, "cmd_lightring", 10)
        self.hazard_subscriber = self.create_subscription(HazardDetection, 'hazard_detection', self.hazard_detect_cb, 10)
        self.hazard_subscriber # prevent unused variable warning

        self.run_rainbow = False
        self.rainbow_thread = None

        self.twist_msg = Twist()
        self.led_msg = LightringLeds()
        
        self.led_msg.override_system = True

        self.controller = controller
        
    def start_up_controller(self):
        self.controller.register_cmd_vel_pub_cb(self.publish_twist_msg)
        self.controller.register_cmd_led_pub_cb(self.toggle_rainbow)
        self.ct = threading.Thread(target=self.controller.spin_controller)
        self.ct.start()
    
    def publish_twist_msg(self, drive, turn):
        self.twist_msg.linear.x = float(drive)
        self.twist_msg.angular.z = float(turn)
        self.twist_publisher.publish(self.twist_msg)
    
    def toggle_rainbow(self):
        self.run_rainbow = not self.run_rainbow
        if self.run_rainbow:
            self.rainbow_thread = threading.Thread(target=self.circle_rainbow)
            self.rainbow_thread.start()
        elif self.rainbow_thread != None:
            self.rainbow_thread.stop()
            self.rainbow_thread = None

    # https://github.com/ipekgoktan/SHINE_Ipek-Mena/blob/master/src/kuri_mi/src/kuri_light.py
    def circle_rainbow(self):
        # Tunable rainbow params
        center = 128
        width = 127
        length = 50
        f1 = f2 = f3 = 0.3
        p1 = 0
        p2 = 2
        p3 = 4

        # rainbow pattern
        for i in range(length):
            for p in range(len(self.led_msg.leds)):
                self.led_msg.leds[p].red = math.sin(f1*i + p1) * width + center
                self.led_msg.leds[p].green = math.sin(f2*i + p2) * width + center
                self.led_msg.leds[p].blue = math.sin(f3*i + p3) * width + center
            self.publish_led()
            time.sleep(0.1)

    def publish_led(self):
        self.led_publisher.publish(self.led_msg)

    def hazard_detect_cb(self, msg):
        print(f"Hazard detected: {msg.type}") #https://github.com/iRobotEducation/irobot_create_msgs/blob/rolling/msg/HazardDetection.msg


def main():
    rclpy.init()
    controller = PS4Controller(
        interface="/dev/input/js0", connecting_using_ds4drv=False
    )
    create_controller_pub = CreateControllerPub(controller)
    create_controller_pub.start_up_controller()
    rclpy.spin(create_controller_pub)

    create_controller_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

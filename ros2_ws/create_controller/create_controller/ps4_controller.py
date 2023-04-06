from pyPS4Controller.controller import Controller
import pigpio
import time

GPIO = pigpio.pi()


class PS4Controller(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.cmd_vel_pub_cb = None
        self.cmd_led_pub_cb = None
        self.x = 0
        self.y = 0
        self.pwm_feq = 20000

        self.l_paddle_pin = 12 # set to whatever the left bumper pin is
        self.r_paddle_pin = 13 # set to whatever the right bumper pin is
        GPIO.set_mode(self.l_paddle_pin, pigpio.OUTPUT)
        GPIO.set_mode(self.r_paddle_pin, pigpio.OUTPUT)

        GPIO.hardware_PWM(self.l_paddle_pin, self.pwm_feq, 0)
        GPIO.hardware_PWM(self.r_paddle_pin, self.pwm_feq, 0)

        GPIO.set_PWM_frequency(self.l_paddle_pin, self.pwm_feq)
        GPIO.set_PWM_frequency(self.r_paddle_pin, self.pwm_feq)
        GPIO.set_PWM_dutycycle(self.l_paddle_pin, 0)
        GPIO.set_PWM_dutycycle(self.r_paddle_pin, 0)

    
    # ROS callbacks
    def register_cmd_vel_pub_cb(self, callback_func):
        self.cmd_vel_pub_cb = callback_func
    
 #   def register_cmd_led_pub_cb(self, callback_func):
#        self.cmd_led_pub_cb = callback_func

    def pub_movement(self):
        self.cmd_vel_pub_cb(self.y, self.x)

    # Listen
    def spin_controller(self):
        self.listen(on_connect=self.connected, on_disconnect=self.disconnected)

    # Controller events
    def on_R3_left(self, value):
        self.move_robot_X(-value)

    def on_R3_right(self, value):
        self.move_robot_X(-value)

    def on_L3_up(self, value):
        self.move_robot_Y(-value)

    def on_L3_down(self, value):
        self.move_robot_Y(-value)

    def on_L2_press(self, value):
        print(f"L2 Raw: {value}")
        self.send_pwm_left(value)

    def on_R2_press(self, value):
        print(f"R2 Raw: {value}")
        self.send_pwm_right(value)
    
    def on_R2_release(self):
        GPIO.set_PWM_dutycycle(self.r_paddle_pin, 190)

    def on_L2_release(self):
        GPIO.set_PWM_dutycycle(self.l_paddle_pin, 190)

    # def on_L3_y_at_rest(self):
    #     self.move_robot_Y(0)

    # def on_R3_x_at_rest(self):
    #     self.move_robot_X(0)
    
   # def on_x_press(self):
    #    self.cmd_led_pub_cb()
    
    # Mapping
    def map_movement(self,value):  # function that maps raw joystick data  --> create angle
        return (float(value) / 32767.0) * 0.46

    def map_motor(self,value):
        value = int((((float(value) + 32767.0) // 256)+ 190) * .573)
        print("NEW VALUE 190-255: ", value, "\n")
        return value

    # Actions
    def move_robot_X(self,value):
        self.x = self.map_movement(value)
        print(f"X: {self.x}")
        self.pub_movement()

    def move_robot_Y(self,value):
        self.y = self.map_movement(value)
        print(f"Y: {self.y}")
        self.pub_movement()

    def send_pwm_left(self,value):		# Function to send PWM FLIPPER signals to GPIO pin for Flipper on left (PIN y)
        value = self.map_motor(value)
        print(f"LT duty value: {value}")
        GPIO.set_PWM_dutycycle(self.l_paddle_pin, value)


    def send_pwm_right(self,value):		# Function to send PWM FLIPPER signals to GPIO pin for Flipper on right(PIN x)
        value = self.map_motor(value)
        print(f"RT duty value: {value}")
        GPIO.set_PWM_dutycycle(self.r_paddle_pin, value)
    
    def reset_flippers(self):  # Function to slowly rotate flippers back so they can synchronize for next shot (Good for Accuracy)
        pass
    
    # Connect/Disconnect
    def connected(self):
        print(f"Controller connected")
    
    def disconnected(self):
        print(f"Controller disconnected")
    


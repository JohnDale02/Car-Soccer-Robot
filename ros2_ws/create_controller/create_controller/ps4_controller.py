from pyPS4Controller.controller import Controller
import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(ledpin,GPIO.OUT)

class PS4Controller(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.cmd_vel_pub_cb = None
        self.cmd_led_pub_cb = None
        self.x = 0
        self.y = 0

        # pwm
        self.l_paddle_pin = 12 # set to whatever the left bumper pin is
        self.r_paddle_pin = 13 # set to whatever the right bumper pin is
        self.pwm_feq = 100 # set desired frequency, range [10-150Hz]
        self.l_pi_pwm = GPIO.PWM(self.l_paddle_pin, self.pwm_feq) #create PWM instance with frequency
        self.r_pi_pwm = GPIO.PWM(self.r_paddle_pin, self.pwm_feq) #create PWM instance with frequency

    
    # ROS callbacks
    def register_cmd_vel_pub_cb(self, callback_func):
        self.cmd_vel_pub_cb = callback_func
    
    def register_cmd_led_pub_cb(self, callback_func):
        self.cmd_led_pub_cb = callback_func

    def pub_movement(self):
        self.cmd_vel_pub_cb(self.y, self.x)

    # Listen
    def spin_controller(self):
        self.pi_pwm.start(0)				#start PWM of required Duty Cycle 
        self.listen(on_connect=self.connected, on_disconnect=self.disconnected)

    # Controller events
    def on_R3_left(self, value):
        self.move_robot_X(value)

    def on_R3_right(self, value):
        self.move_robot_X(value)

    def on_L3_up(self, value):
        self.move_robot_Y(value)

    def on_L3_down(self, value):
        self.move_robot_Y(value)

    def on_L2_press(self, value):
        self.send_pwm_left(value)

    def on_R2_press(self, value):
        self.send_pwm_right(value)

    # def on_L3_y_at_rest(self):
    #     self.move_robot_Y(0)

    # def on_R3_x_at_rest(self):
    #     self.move_robot_X(0)
    
    def on_x_press(self):
        self.cmd_led_pub_cb()
    
    # Mapping
    def map_movement(self,value):  # function that maps raw joystick data  --> create angle
        return (float(value) / 32767.0) * 0.46

    def map_motor(self,value):
        return abs(float(value) / 32767.0 * 75)

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
        self.l_pi_pwm.ChangeDutyCycle(value)


    def send_pwm_right(self,value):		# Function to send PWM FLIPPER signals to GPIO pin for Flipper on right(PIN x)
        value = self.map_motor(value)
        print(f"RT duty value: {value}")
        self.r_pi_pwm.ChangeDutyCycle(value)
    
    def reset_flippers(self):  # Function to slowly rotate flippers back so they can synchronize for next shot (Good for Accuracy)
        pass
    
    # Connect/Disconnect
    def connected(self):
        print(f"Controller connected")
    
    def disconnected(self):
        print(f"Controller disconnected")
    


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
        self.IOLEFT = 0
        self.IORIGHT = 0
        self.MarioMode = True

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

    def pub_movement(self):
        self.cmd_vel_pub_cb(self.y, self.x)

    # Listen
    def spin_controller(self):
        self.listen(on_connect=self.connected, on_disconnect=self.disconnected)

    # Controller events
    def on_R3_left(self, value):
        if self.MarioMode == False:
            self.move_robot_X(-value)

    def on_left_arrow_press(self):
        if self.MarioMode == True:
            print("Turning left")
            self.x = 0.7

    def on_R3_right(self, value):
        if self.MarioMode == False:
            self.x = -.7
        
    def on_right_arrow_press(self):
        if self.MarioMode == True:
            print("Turning right")
            self.x = -0.7
    
    def on_left_right_arrow_release(self):
        print("Stopping Turn")
        self.x = 0.0

    def on_L3_up(self, value):
        if self.MarioMode == False:
            self.move_robot_Y(-value)

    def on_L3_down(self, value):
        if self.MarioMode == False:
            self.move_robot_Y(-value)

    def on_L1_press(self): 
        GPIO.set_mode(self.l_paddle_pin, pigpio.OUTPUT)
        print("on_L1_press: ", self.IOLEFT) 
        if self.IOLEFT == 0:
            GPIO.set_PWM_dutycycle(self.l_paddle_pin, 0)
        if self.IOLEFT == 1:
            GPIO.set_PWM_dutycycle(self.l_paddle_pin, 255)

    def on_L1_release(self): # stop to motor and IOLEFT to opposite
        GPIO.set_mode(self.l_paddle_pin, pigpio.INPUT)
        print("L1 Release: ", self.IOLEFT)
        if self.IOLEFT == 0:
            GPIO.set_PWM_dutycycle(self.r_paddle_pin, 190)
            self.IOLEFT = 1
        elif self.IOLEFT == 1:
            GPIO.set_PWM_dutycycle(self.r_paddle_pin, 190)
            self.IOLEFT = 0
        print(self.IOLEFT)

    def on_R1_press(self): 
        GPIO.set_mode(self.r_paddle_pin, pigpio.OUTPUT)
        print("on_R1_press: ", self.IORIGHT) 
        if self.IORIGHT == 0:
            GPIO.set_PWM_dutycycle(self.r_paddle_pin, 255)
        if self.IORIGHT == 1:
            GPIO.set_PWM_dutycycle(self.r_paddle_pin, 0)

    def on_R1_release(self): # stop to motor and IOLEFT to opposite
        print("R1 Release: ", self.IORIGHT)
        if self.IORIGHT == 0:
            GPIO.set_PWM_dutycycle(self.r_paddle_pin, 170)
            self.IORIGHT = 1
        elif self.IORIGHT == 1:
            GPIO.set_PWM_dutycycle(self.r_paddle_pin, 170)
            self.IORIGHT = 0  
        print(self.IORIGHT)

    def on_L3_y_at_rest(self):
        if self.MarioMode == False:
            self.y = 0.0

    def on_R3_x_at_rest(self):
        if self.MarioMode == False:
            self.x = 0.0

    def on_triangle_press(self):
        if self.MarioMode == True:
            self.MarioMode = False
            print("Mario Mode disabled")
            time.sleep(.5)
            
        elif self.MarioMode == False:
            self.MarioMode = True
            print("Mario Mode enabled")
            time.sleep(.5)
            
    def on_circle_press(self):
        if self.MarioMode == True:
            self.y = 0.46 # go full speed forwards

    def on_circle_release(self):
        self.y = 0.0

    def on_x_press(self):
        if self.MarioMode == True:
            self.y = -0.46  # Go max speed backwards

    def on_x_release(self):
        self.y = 0.0
    
    # Mapping
    def map_movement(self,value):  # function that maps raw joystick data  --> create angle
        return (float(value) / 32767.0) * 0.46

    def map_motor(self,value):
        value = (float(value) + 32767.0) // 256
        print(value, "\n")
        return value

    # Actions
    def move_robot_X(self,value):
        self.x = self.map_movement(value)
        print(f"X: {self.x}")
        #self.pub_movement()

    def move_robot_Y(self,value):
        self.y = self.map_movement(value)
        print(f"Y: {self.y}")
        #self.pub_movement()

    def send_pwm_left(self,value):		# Function to send PWM FLIPPER signals to GPIO pin for Flipper on left (PIN y)
        value = self.map_motor(value)
        print(f"LT duty value: {value}")
        GPIO.set_PWM_dutycycle(self.l_paddle_pin, value)

    def send_pwm_right(self,value):		# Function to send PWM FLIPPER signals to GPIO pin for Flipper on right(PIN x)
        value = self.map_motor(value)
        print(f"RT duty value: {value}")
        GPIO.set_PWM_dutycycle(self.r_paddle_pin, value)
    
    # Connect/Disconnect
    def connected(self):
        print(f"Controller connected")
    
    def disconnected(self):
        print(f"Controller disconnected")
    


from pyPS4Controller.controller import Controller

class PS4Controller(Controller):
	def __init__(self, **kwargs):
		Controller.__init__(self, **kwargs)

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

	def on_L3_y_at_rest(self):
		self.move_robot_Y(0)

	def on_R3_x_at_rest(self):
		self.move_robot_X(0)
	
	def map_movement(self,value):  # function that maps raw joystick data  --> create angle
		return value // 100

	def map_motor(self,value):
		return value // 100

	def move_robot_X(value):
		value = self.map_movement(value)
		print(f"X: {value}")

	def move_robot_Y(value):
		value = self.map_movement(value)
		print(f"Y: {value}")

	def send_pwm_left(value):		# Function to send PWM FLIPPER signals to GPIO pin for Flipper on left (PIN y)
		value = self.map_motor(value)
		print(f"LT: {value}")

	def send_pwm_right(value):		# Function to send PWM FLIPPER signals to GPIO pin for Flipper on right(PIN x)
		value = self.map_motor(value)
		print(f"RT: {value}")
	
	def reset_flippers():  # Function to slowly rotate flippers back so they can synchronize for next shot (Good for Accuracy)
		pass

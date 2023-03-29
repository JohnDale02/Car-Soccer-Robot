from pyPS4Controller.controller import Controller
# I have altered the pyPS4 Controller package by commenting out some events so they arent being registered for buttons we arent using or joystick axis we dont care about
# the updated pyPS4Controller controller file is in the directory too, not sure if this needs to be edited / how much processing time it saves us but thought it couldnt hurt

previous_X = 0
previous_Y = 0

def map_movement(value):  # function that maps raw joystick data  --> create angle
	value = value // 100
	print(value, ": mapping")
	return value

def map_motor(value):
	value = value // 100
	print(value, ": mapping")
	return value

def move_robot_X(value):
	print("Moving Robot X: ", "[",value,",", previous_Y,"]")

def move_robot_Y(value):
	print("Moving Robot Y: ", "[",previous_X,",",  value, "]")

def send_pwm_left(value):		# Function to send PWM FLIPPER signals to GPIO pin for Flipper on left (PIN y)
	pass

def send_pwm_right(value):		# Function to send PWM FLIPPER signals to GPIO pin for Flipper on right(PIN x)
	pass

def reset_flippers():  # Function to slowly rotate flippers back so they can synchronize for next shot (Good for Accuracy)
	pass


class MyController(Controller):
	def __init__(self, **kwargs):
		Controller.__init__(self, **kwargs)
		#self.black_listed_buttons = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17]

#	def on_L3_x_at_rest(self):
#		global previous_X
#		previous_X = 0

#	def on_L3_y_at_rest(self):
#		global previous_Y
#		previous_Y = 0

	def on_R3_left(self, value):
		global previous_X
		value = map_movement(value)
		previous_X = value
		move_robot_X(value)

	def on_R3_right(self, value):
		global previous_X
		value = map_movement(value)
		previous_X = value
		move_robot_X(value)

	def on_L3_up(self, value):
		global previous_Y
		value = map_movement(value)
		previous_Y = value
		move_robot_Y(value)

	def on_L3_down(self, value):
		global previous_Y
		value = map_movement(value)
		previous_Y = value
		move_robot_Y(value)

	def on_L2_press(self, value):
		value = map_motor(value)
		send_pwm_left(value)

	def on_R2_press(self, value):
		value = map_motor(value)
		send_pwm_right(value)

	def on_L3_y_at_rest(self):
		global previous_Y
		previous_Y = 0
		value = 0
		move_robot_Y(value)

	def on_R3_x_at_rest(self):
		global previous_X
		previous_X = 0
		value = 0
		move_robot_X(value)

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen()



""" 
-------------------------------------------------------
Name: PIDcontrol.py
Creator:  Chris Vail, Abi Langbridge, Matt Ryan
Date:   March 2020
Revision: 1.0
-------------------------------------------------------
"""

from pyb import Pin, Timer, ADC, UART, Switch, millis, LED, delay
from motor import DriveChain, Motor
from PIDcontrol import ControlSystem, PIDcontroller
from mpu6050 import MPU6050
from oled_938 import OLED_938
from mic import Microphone
import micropython
micropython.alloc_emergency_exception_buf(100)


class Segway(object):

	def __init__(self, drive, microphone, uart, control_system, oled, pot, switch, choreo):
		# Initialise drive control
		self.control = control_system
		self.drive = drive
		self.uart_drive = 0
		self.uart = uart
		self.turn = 0
		# Initialise choreographer
		self.choreo = choreo
		# Initialise microphone
		self.mic = microphone
		# Initialise tuning system
		self.pot = pot
		self.oled = oled
		self.switch = switch
		# Prep and use OLED display
		oled.poweron()
		oled.init_display()
		oled.draw_text(0,0, 'Segway Initialised')
		oled.display()

		# Tune the pid controller
		self.tune_control()

			
	def tune_control(self):
        """ Use pot to tune each parameter k_p, k_i, k_d in turn
            Press button on pyboard to change to next parameter """
		# UGLY function, needs serious refactoring
		while self.switch(): pass
		self.oled.clear()
		self.oled.draw_text(0,0 , "Tuning PID control:")
		while not self.switch():
			self.drive.drive(self.control.update())
			self.control.pid.k_p = self.pot.read()*8/4095
			self.oled.draw_text(0,30 , 'k_p = {:5.2f}'.format(self.control.pid.k_p))
			self.oled.draw_text(0, 45, "theta = {:5.2f}".format(self.control.pid.angle))
			self.oled.display()
			pyb.delay(25)

		while self.switch(): pass

		self.oled.clear()
		self.oled.draw_text(0,0 , "Tuning PID control:")
		while not self.switch():
			self.drive.drive(self.control.update())
			self.control.pid.k_i = self.pot.read()*20/4095
			self.oled.draw_text(0,30 , 'k_i = {:5.2f}'.format(self.control.pid.k_i))
			self.oled.display()
			pyb.delay(25)
		
		while self.switch(): pass

		self.oled.clear()
		self.oled.draw_text(0,0 , "Tuning PID control:")
		while not self.switch():
			self.drive.drive(self.control.update())
			self.control.pid.k_d = self.pot.read()*2/4095
			self.oled.draw_text(0,30 , 'k_d = {:5.2f}'.format(self.control.pid.k_d))
			self.oled.display()
			pyb.delay(25)

		self.oled.clear()
		self.oled.display()

	def uart_command(self):
        """ Process uart commands to adjust setpoint and turning """
		if self.uart.any() == 10: 
			# Process the command
			command = self.uart.read(10)
			print(command[2])
			if command[2] == 53:
				self.control.increment_setpoint(5)
				#self.uart_drive += 10
			elif command[2] == 54:
				self.control.increment_setpoint(-5)
				#self.uart_drive += -10
			elif command[2] == 55:
				self.turn += -10
			elif command[2] == 56:
				self.turn += 10

	
	def main_loop(self):
        # LED for flashing on beat detect
		led = LED(1)
		
		while True:
			if self.mic.is_beat(500):		
				led.on()
				delay(50)
				led.off()
                # Adjust setpoint based on next dance move if there is a beat
				setpoint, self.turn = self.choreo.next_move()
				self.control.update_setpoint(setpoint)
				#self.uart_drive = setpoint
					
			#self.uart_command()
            # Use control system to drive the motors
			self.drive.drive(self.control.update()) # + self.uart_drive)
            # Adjust for turning
			self.drive.turn(self.turn)
			

class Choreographer(object):

	def __init__(self, filepath):
		# Assume filepath exists
        # Reads and decodes dance moves on initialisation
		with open(filepath, 'r') as f:
			self.move_list = [self.decode_move(m) for m in f.readlines()]
		self.ptr = 0

	def decode_move(self, move):
        # Decode moves based on simple encoding
		speed = int(move[1])*20 if move[0] == "F" else -int(move[1])*20
		turn = int(move[3])*20 if move[0] == "R" else -int(move[3])*10
		return speed, turn

	def next_move(self):
        # Gives moves until out and then gives (0, 0)
		if self.ptr < len(self.move_list):
			move = self.move_list[self.ptr]
			self.ptr += 1
		else:
			move = (0, 0)
		return move
	



def main():


	# Initialise motors with required pins and PWM timers
    # Motors collected in DriveChain object
	timer = Timer(2, freq=1000)
	motorA = Motor(Pin('X3', Pin.OUT_PP), Pin('X4', Pin.OUT_PP), Pin('X1'), timer, 1)
	motorB = Motor(Pin('X8', Pin.OUT_PP), Pin('X7', Pin.OUT_PP), Pin('X2'), timer, 2)
	drive = DriveChain(motorA, motorB)

	# Microphone
	mic_timer = Timer(4, freq=8000)
	mic = Microphone(mic_timer, ADC(Pin("Y11", Pin.IN)), 64)

	# Initialise UART channel
	uart = UART(6)
	uart.init(9600, bits=8, parity=None, stop=2)

	# Initialise control systems
	pid = PIDcontroller(0, 0, 0, clamp_lim=[-75, 75], set_point=2)
	imu = MPU6050(1, False)    	# Use I2C port 1 on Pyboard
	control = ControlSystem(pid, imu)

	# Initialise oled display
	# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
	oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64,
					external_vcc=False, i2c_devid=61)

	# Hardware for tuning pid
	pot = ADC(Pin("X11"))
	switch = pyb.Switch()

	# Collect dance moves
	choreo = Choreographer("new_dance.txt")

    # Create segway object and run main loop
	segway = Segway(drive, mic, uart, control, oled, pot, switch, choreo)
	segway.main_loop()


if __name__ == "__main__":
	main()
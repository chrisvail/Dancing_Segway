
""" 
-------------------------------------------------------
Name: motor.py
Creator:  Chris Vail, Abi Langbridge, Matt Ryan
Date:   March 2020
Revision: 1.0
-------------------------------------------------------
Micropython motor drive module.
------------------------------------------------------- 
"""


import pyb, micropython
from pyb import Pin, Timer


class Motor(object):

	def __init__(self, pin1, pin2, PWM_pin, timer, channel):

		# h bridge rails
		self.pin1 = pin1
		self.pin2 = pin2

		# PWM control
		self.PWM_pin = PWM_pin
		self.timer = timer
		self._motor = self.timer.channel(channel, Timer.PWM, pin=PWM_pin)

		# initialise speed to 0
		self.speed = 0

	# drives motor at a speed passed to function
	# validated and motor's PWM timer updated
	# h-bridge control used to define direction
	def drive(self, speed):
		self.validate_speed(speed)

		if self.speed > 0:
			self.pin1.low()
			self.pin2.high()
			self._motor.pulse_width_percent(min(100, self.speed))

		elif self.speed < 0:
			self.pin1.high()
			self.pin2.low()
			self._motor.pulse_width_percent(min(100, abs(self.speed)))

		else:
			self._motor.pulse_width_percent(0)
	
	# increments or decrements current motor speed
	def increment_speed(self, change):
		self.drive(self.speed + change)

	# stops motor
	def stop(self):
		self.speed = 0
		self._motor.pulse_width_percent(0)

	# checks speed lies between -100 and 100
	# to ensure PWM drive compatibility
	def validate_speed(self, speed):
		if speed > 100: 
			self.speed = 100

		elif speed < -100: 
			self.speed = -100

		else:
			self.speed = speed


class DriveChain(object):

	def __init__(self, right_motor, left_motor):
		self.left_motor = left_motor
		self.right_motor = right_motor

		# ensure both motors are stopped
		self.right_motor.drive(0)
		self.left_motor.drive(0)

	# uniform drive - should yield a straight
	# path either forwards or backwards
	def drive(self, speed):
		self.right_motor.drive(speed)
		self.left_motor.drive(speed)
	
	# uniform incrementation of motor speeds
	def increment_speed(self, change):
		self.right_motor.increment_speed(change)
		self.left_motor.increment_speed(change)

	# method to control turning of the segway
	# by controlling motor speeds independently
	# positive turn_rate is a right turn
	def turn(self, turn_rate):
		if turn_rate > 0:
			self.left_motor.increment_speed(turn_rate)

		elif turn_rate < 0:
			self.right_motor.increment_speed(-turn_rate)
	
	# another turning method which produces a more
	# rapid spin by updating both motor_speeds
	# again, positive twist_rate yields right turn
	def twist(self, twist_rate):
		self.left_motor.increment_speed(twist_rate)
		self.right_motor.increment_speed(-twist_rate)

	# method to straighten the drive path by setting
	# both motors to the same speed  
	def straighten(self):
		self.drive((self.left_motor.speed  + self.right_motor.speed)//2)

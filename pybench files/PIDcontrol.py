""" 
-------------------------------------------------------
Name: PIDcontrol.py
Creator:  Chris Vail, Abi Langbridge, Matt Ryan
Date:   March 2020
Revision: 1.0
-------------------------------------------------------
PID controller with integral windup protection
and low pass filtering on the derivative.
------------------------------------------------------- 
"""

from pyb import millis

class PIDcontroller(object):

	def __init__(self, k_p, k_d, k_i, clamp_lim=None, set_point=0):
		# initialise weightings
		self.k_p = k_p
		self.k_i = k_i
		self.k_d = k_d

		# intitialise setpoint
		self.setpoint = set_point

		# initialise derivative variables
		self.derivative = 0
		self.old_error = 0
		self.angle = 0
		
		# initialise integration variables
		self.integrator = 0
		self.clamping = False

		if clamp_lim is not None:
			self.clamp_upper = clamp_lim[1]
			self.clamp_lower = clamp_lim[0]
		else:
			self.clamp_upper = 999999999
			self.clamp_lower = -999999999
		
	# updates angle error, returns sum of error components
	# if clamping is enabled, raises necessary flag 
	def __call__(self, reading, dt, theta_dot):
		self.angle = reading
		error = self.setpoint - reading
		self.old_error = error

		rtn = sum((self.k_p*error, 
				   self.k_d*theta_dot, 
				   self.k_i*self.get_integral(error, dt)))

		if rtn > self.clamp_upper: 
			self.clamping = "U"
		elif rtn < self.clamp_lower:
			self.clamping = "L"

		return rtn

	# method to allow manual, real-time tuning of
	# PID weightings 
	def tune(self, pot, switch, oled):
		state = switch()

		while state == switch():
			self.k_p = pot.read()*6/4095
			oled.draw_text(0,30 , 'k_p = {:5.2f}'.format(self.k_p))
			oled.display()
		
		while state != switch():
			self.k_i = pot.read()*20/4095
			oled.draw_text(0,30 , 'k_i = {:5.2f}'.format(self.k_i))
			oled.display()
		
		while state == switch():
			self.k_d = pot.read()*2/4095
			oled.draw_text(0,30 , 'k_d = {:5.2f}'.format(self.k_d))
			oled.display()

	# returns derivative calculated using
	# change in error over timestep
	def get_derivative(self, error, dt):
		self.derivative = (error - self.old_error)/dt
		return self.derivative

	# returns integral calculated by accumulation
	def get_integral(self, error, dt):
		if not self.is_clamping(error):
			self.integrator += error*dt
		return self.integrator

	# checks error falls inside clamping bounds
	def is_clamping(self, error):
		if self.clamping == "U" and error > 0:
			return True
		elif self.clamping == "L" and error < 0:
			return True
		else:
			return False


class ControlSystem(object):

	def __init__(self, pid_controller, imu, ALPHA=0.95):
		self.pid = pid_controller
		self.imu = imu
		self.ALPHA = ALPHA
		self.pitch = 0
		self.last_call = millis()	

	# get new value for position using complementary
	# filter function, feed to PID
	def update(self):
		dt = (millis() - self.last_call)*0.001
		# catch for dt misrepresented as 0
		if dt <= 0: dt = 0.001

		self.last_call = millis()
		theta_dot = self.imu.get_gy()
		self.pitch = self.ALPHA*(self.pitch + theta_dot*dt) + (1 - self.ALPHA)*self.imu.pitch()

		return self.pid(self.pitch, dt, theta_dot)

	# increment the setpoint
	def increment_setpoint(self, change):
		self.pid.setpoint += change

	# modify the setpoint to a value
	def update_setpoint(self, point):
		self.pid.setpoint = point
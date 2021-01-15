'''
-------------------------------------------------------
Name: microphone 
Creator:  Chris Vail, Abi Langbridge, Matt Ryan
Date:   March 2020
Revision: 1.0 
-------------------------------------------------------
Acquire real-time data from microphone at 8kHz sampling frequency
Find energy in 20msec window

-------------------------------------------------------
'''
import pyb
from pyb import Pin, ADC, Timer, millis
from array import array

#  The following two lines are needed by micropython 
#   ... must include if you use interrupt in your program
import micropython
micropython.alloc_emergency_exception_buf(100)

class Microphone(object):

	def __init__(self, timer, mic, sample_length, MIC_OFFSET=1523):
		# ADC reading of microphone for silence
		self.MIC_OFFSET = MIC_OFFSET	
		# initialise variables used for beat detection
		self.sample_length = sample_length
		self.sample_count = 0
		# Energy values
		self.energy = 0
		self.buffer = array("I", 0 for i in range(16))  # Array of uints
		self.ptr = 0
		self._cumulating_energy = 0
		# Beat detection
		self.last_call = 0
		self.threshold = 3.6
		# Hardware input
		self.mic = mic
		# Specify timer interrupt service routine
		timer.callback(self.isr_sampling)

	# Interrupt service routine to fill 
	def isr_sampling(self, line): 
        # read one sample and remove dc offset
		sample = self.mic.read() - self.MIC_OFFSET  
		self.sample_count += 1
        # Add to energy summation
		self._cumulating_energy += sample*sample
		if (self.sample_count == self.sample_length):
			self.sample_count = 0
            # Update values with new energy value and reset for next calc
			self.buffer[self.ptr], self.energy, self._cumulating_energy = self.energy, self._cumulating_energy, 0
			self.ptr = (self.ptr + 1) % 16

	
	def is_beat(self, beat_dist=0):
        # Check for beat after a time limit in ms
		if self.energy*16 > sum(self.buffer)*self.threshold and millis()-self.last_call > beat_dist:
			print(millis() - self.last_call)
            # Store time of last beat
			self.last_call = millis()
			return True
		return False
			

		
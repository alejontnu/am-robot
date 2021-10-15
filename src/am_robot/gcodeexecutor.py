import os
import time
from gcodeparser import GcodeParser

import am_robot

class GCodeExecutor:
	'''
	Read and parse gcode into a full object variable
	
	Attributes:
	-----------



	Methods:
	-----------

	'''
	def __init__(self,filename):
		initial_interval = [0,0]
		self.interval = initial_interval

		file_extension = '.gcode'
		if file_extension not in filename:
			filename = filename + '.gcode'
		folder = 'data'
		fullpath = os.path.join('.',folder,filename)

		with open(fullpath,'r') as file:
			self.gcodelines = GcodeParser(file.read()).lines

		# scan through sections at a time
		self.filename = filename
		self.number_of_lines = len(self.gcodelines)

	def get_interval(self):
		return self.interval
	
	# Set the current interval of gcode lines
	def set_interval(self,interval):
		self.interval = interval

	def get_command(self,line_number):
		return self.gcodelines[line_number].command[0] + str(self.gcodelines[line_number].command[1])

	def set_command(self):
		interval = self.interval
		command = self.get_command(interval[0])
		self.command = command

	def get_params(self,line_number,**param):
		return 0

	def make_waypoints(self):
		return 0

	def run_code_segment(self):
		command = self.command
		return 0

	# Find the next instance of retraction (NB may be 0mm retraction)
	def find_interval(self):
		previous_interval = self.interval

		if previous_interval[1]+1 >= self.number_of_lines:
			return 'End of code' 
		
		counter = previous_interval[1]+1
		while (counter < self.number_of_lines):
			if self.get_command(counter) != self.get_command(previous_interval[1]+1):
				interval = [previous_interval[1]+1,counter-1]
				print(interval)
				self.set_interval(interval)
				return interval
			elif self.get_params(counter) != self.get_params(previous_interval[1]+1):
				interval = [previous_interval[1]+1,counter-1]
				print(interval)
				self.set_interval(interval)
				return interval
			else:
				counter = counter + 1

		
		print(interval)
		self.set_interval(interval)

	def __str__(self):
		return "gcodeexecutor"

	def display(self):
		print(f"Name of file being processed: {self.filename}")
		print(f"Current interval: {self.interval}")


if __name__ == '__main__':
	filename = 'Circle'
	executioner = GCodeExecutor(filename)
	executioner.find_interval()
	executioner.find_interval()
	executioner.find_interval()
	executioner.find_interval()
	executioner.find_interval()
	executioner.find_interval()
	executioner.find_interval()
	executioner.find_interval()
	executioner.find_interval()
	executioner.find_interval()
	executioner.find_interval()
	executioner.find_interval()
	executioner.find_interval()
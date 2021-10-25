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


		file_extension = '.gcode'
		if file_extension not in filename:
			filename = filename + '.gcode'
		folder = 'data'
		fullpath = os.path.join('.',folder,filename)

		with open(fullpath,'r') as file:
			self.gcodelines = GcodeParser(file.read()).lines

		initial_interval = [0,0]
		self.interval = initial_interval
		self.list_of_intervals = []
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

	def get_params(self,line_number,param):
		if param in self.gcodelines[line_number].params:
			return self.gcodelines[line_number].params[param]
		else:
			return 0

	def set_params(self,line_number,param):
		self.__dict__[param] = self.get_params(line_number,param)

	def make_waypoints(self):
		return 0

	def run_code_segment(self):
		command = self.command
		return 0

	# Find the next instance of retraction (NB may be 0mm retraction)
	def find_next_interval(self):
		previous_interval = self.interval

		if previous_interval[1]+1 >= self.number_of_lines:
			print("End of code")
			return 'End of code' 
		
		counter = previous_interval[1]+1
		interval = [counter,counter]
		while (counter < self.number_of_lines):
			if self.get_command(counter) != self.get_command(previous_interval[1]+1):
				interval = [previous_interval[1]+1,counter-1]
				print("interval end due to command change")
				break
			elif self.get_params(counter,'F') != 0 and self.get_params(counter,'F') != self.F:
				interval = [previous_interval[1]+1,counter-1]
				self.set_params(counter,'F')
				print("interval end due to F param change")
				break
			elif self.get_params(counter,'E') < self.get_params(counter-1,'E'):
				interval = [previous_interval[1]+1,counter-1]
				self.set_params(counter,'E')
				print("interval end due to E param change")
				break
			else:
				counter = counter + 1

		self.set_interval(interval)
		self.append_interval()


	def append_interval(self):
		self.list_of_intervals.append(self.interval)

	def __str__(self):
		return "gcodeexecutor"

	def display(self):
		print(f"Name of file being processed: {self.filename}")
		print(f"Current interval: {self.interval}")

	def visualize_gcode(self):
		return 0


if __name__ == '__main__':
	filename = 'Circle'
	executioner = GCodeExecutor(filename)
	executioner.find_next_interval()
	executioner.find_next_interval()
	executioner.find_next_interval()
	executioner.find_next_interval()
	executioner.find_next_interval()
	executioner.find_next_interval()
	executioner.find_next_interval()
	executioner.find_next_interval()
	executioner.find_next_interval()
	executioner.find_next_interval()
	executioner.find_next_interval()
	executioner.find_next_interval()
	executioner.find_next_interval()
	executioner.find_next_interval()
	executioner.find_next_interval()
	executioner.find_next_interval()

	print("end of intervals")
	print(executioner.list_of_intervals)
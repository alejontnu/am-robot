import os
import time
from gcodeparser import GcodeParser
from frankx import Robot
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import am_robot
import extruder

class GCodeExecutor:
	'''
	Read and parse gcode into a full object variable
	
	Attributes:
	-----------

	Methods:
	-----------

	'''
	def __init__(self,_filename,_robot,_extruder_tool):
		self.filename = _filename
		self.interval = [0,0]
		self.list_of_intervals = []

		# The [minimum,maximum] occurence through gcode
		self.Xmax = [0,0]
		self.Ymax = [0,0]
		self.Zmax = [0,0]
		self.Fmax = [0,0]

		self.E = -100
		self.F = -100
		self.move_type = 'idle'

		self.robot = am_robot
		self.extruder_tool = extruder_tool

	def load_gcode(self):
		filename = self.filename
		file_extension = '.gcode'
		if file_extension not in filename:
			filename = filename + '.gcode'
		folder = 'data'
		fullpath = os.path.join('.',folder,filename)

		with open(fullpath,'r') as file:
			self.gcodelines = GcodeParser(file.read()).lines

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
			return -1

	def set_params(self,line_number,param):
		self.__dict__[param] = self.get_params(line_number,param)

	def set_extremes(self,param,extreme):
		if param > self.__dict__[extreme][1]:
			self.__dict__[extreme][1] = param
		if param < self.__dict__[extreme][0]:
			self.__dict__[extreme][0] = param

	def make_waypoints(self):
		return 0

	def run_code_segment(self):
		command = self.command
		return 0

	# Find the next instance of retraction (NB may be 0mm retraction)
	def find_next_interval(self):
		previous_interval = self.interval
		if self.list_of_intervals == []:
			self.append_interval()

		if self.interval[1]+1 >= self.number_of_lines:
			print("End of code")
			return 'End of code' 
		
		counter = self.interval[1]+1
		interval = [counter,counter]
		while (counter < self.number_of_lines):
			if self.get_params(counter,'F') != -1 and self.get_params(counter,'F') != self.F:
				interval = [self.interval[1]+1,counter-1]
				self.set_params(counter,'F')
				self.set_extremes(self.F,'Fmax')
				#print("interval end due to F param change")
				break
			elif self.get_command(counter) != self.get_command(self.interval[1]+1):
				interval = [self.interval[1]+1,counter-1]
				#print("interval end due to command change")
				break
			elif self.get_params(counter,'E') != -1 and self.get_params(counter,'E') < self.E:
				interval = [self.interval[1]+1,counter-1]
				self.set_params(counter,'E')
				#print("interval end due to E param change")
				break
			else:
				# Typically when the next line is just another X-Y coordinate
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
		layer_height = 0
		extrusion_volume = 0
		x_coordinates = []
		y_coordinates = []
		z_coordinates = []
		F_colours = []
		for element in self.list_of_intervals:
			if self.get_params(element[0],'Z') >= 0:
				layer_height = self.get_params(element[0],'Z')
				self.set_extremes(layer_height,'Zmax')
			if self.get_params(element[0],'E') >= 0:
				extrusion_volume = self.get_params(element[0],'E')
			if self.get_command(element[0]) == 'G1' and element[0] != element[1]:
				for point in range(element[0],element[1]+1):
					if self.get_params(point,'E') > extrusion_volume:
						x = self.get_params(point,'X')
						y = self.get_params(point,'Y')
						x_coordinates.append(x)
						y_coordinates.append(y)
						z_coordinates.append(layer_height)
						self.set_extremes(x,'Xmax')
						self.set_extremes(y,'Ymax')


		fig = plt.figure()
		ax = fig.add_subplot(111,projection='3d')
		ax.set_xlim(self.Xmax[0],self.Xmax[1])
		ax.set_ylim(self.Ymax[0],self.Ymax[1])
		largest_axis = max(self.Xmax[1]-self.Xmax[0],self.Ymax[1]-self.Ymax[0],self.Zmax[1]-self.Zmax[0])
		ax.set_box_aspect(((self.Xmax[1]-self.Xmax[0])/largest_axis,(self.Ymax[1]-self.Ymax[0])/largest_axis,(self.Zmax[1]-self.Zmax[0])/largest_axis))
		ax.plot(x_coordinates,y_coordinates,z_coordinates,label='Visualized gcode model',linewidth=0.05)
		ax.legend()
		plt.show()

		return 0


if __name__ == '__main__':
	robot = {}
	extruder_tool = extruder.ExtruderTool('FDM','10.0.0.3')
	filename = '3DBenchy'

	executioner = GCodeExecutor(filename,robot,extruder_tool)
	executioner.load_gcode()

	count = 0


	while executioner.number_of_lines > executioner.interval[1] + 1 and count < 10000:
		executioner.find_next_interval()
		count = count + 1

	print("end of intervals")
	#print(executioner.list_of_intervals)

	executioner.visualize_gcode()
import os
import time
from gcodeparser import GcodeParser
from frankx import Robot
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import plotly.express as px
import pandas as pd
import plotly.graph_objects as go
import math

import am_robot
import ExtruderTool

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
        self.interval = [0,0] # On the assumption that the first and second gcode command will allways be unique from eachother
        self.list_of_intervals = []

        self.Z = 0
        self.E = -100
        self.F = -100
        self.move_type = 'idle'

        self.robot = _robot
        self.extruder_tool = _extruder_tool

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
        self.find_intervals()

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
            return -100

    def set_params(self,line_number,param):
        self.__dict__[param] = self.get_params(line_number,param)

    def set_extremes(self,param,extreme):
        try:
            if param > self.__dict__[extreme][1]:
                self.__dict__[extreme][1] = param
        except:
            self.__dict__[extreme] = [param,param]
        try:
            if param < self.__dict__[extreme][0]:
                self.__dict__[extreme][0] = param
        except:
            self.__dict__[extreme] = [param,param]

        

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
            if self.get_params(counter,'F') >= 0 and self.get_params(counter,'F') != self.F:
                interval = [self.interval[1]+1,counter-1]
                self.set_params(counter,'F')
                self.set_extremes(self.F,'Fmax')
                #print("interval end due to F param change")
                break
            elif self.get_command(counter) != self.get_command(self.interval[1]+1):
                interval = [self.interval[1]+1,counter-1]
                #print("interval end due to command change")
                break
            elif self.get_params(counter,'E') >= 0 and self.get_params(counter,'E') < self.E:
                interval = [self.interval[1]+1,counter-1]
                self.set_params(counter,'E')
                #print("interval end due to E param change")
                break
            # elif self.get_params(counter,'Z') >= 0 and self.get_params(counter, 'Z' != self.Z):
            #     interval = [self.interval[1]+1,counter-1]
            #     self.set_params(counter,'Z')
            #     #print("interval end due to layer height change")
            #     break
            else:
                # Typically when the next line is just another X-Y coordinate
                counter = counter + 1

        self.set_interval(interval)
        self.append_interval()


    def append_interval(self):
        self.list_of_intervals.append(self.interval)

    def find_intervals(self):
        count = 0
        while self.number_of_lines > self.interval[1] + 1 and count < 100000:
            self.find_next_interval()
            count = count + 1

        self.current_interval = 1

    def make_waypoints(self,interval):
        # Take an interval of movement commands
        # Calculate trapesoidal movement/velocity to reduce jerk and uphold correct extrusion speed
        # ?
        num_waypoints = interval[1]+1-interval[0]
        xyz_coordinates = np.empty([num_waypoints,3])

        for point in range(interval[0],interval[1]+1):
            xyz_coordinates[[point][0]] = self.get_params(point,'X')
            xyz_coordinates[[point][1]] = self.get_params(point,'Y')
            xyz_coordinates[[point][2]] = self.Z
        return 0

    # Blocking action
    def run_code_segment(self):
        print("run code segment")
        interval = self.list_of_intervals[self.current_interval]
        command = self.get_command(interval[0])
        if self.get_params(interval[0]) >= 0:
            self.Z = self.get_params(interval[0])
        if command == 'G0':
            # Stop extrusion and move to target
            fem = 4
        elif command == 'G1':
            # Find waypoints, trapesoidal movement?
            waypoints = self.make_waypoints(self.list_of_intervals[interval_number])
        # check bounds?
        # set extrusion speed
        # feed waypoints to robot and listen to robot pose

        return 0

    def __str__(self):
        return "gcodeexecutor"

    def display(self):
        if '.gcode' in self.filename:
            print(f"\nName of file being processed: {self.filename}")
        else:
            print(f"\nName of file being processed: {self.filename + '.gcode'}")
        print(f"Number of command lines processed: {self.number_of_lines}")
        print("Total filament used: \n")

    def visualize_gcode(self):
        layer_height = 0
        extrusion_volume = 0
        current_feedrate = 0
        x_coordinates = []
        y_coordinates = []
        z_coordinates = []
        colors = []
        for element in self.list_of_intervals:
            if self.get_params(element[0],'F') >= 0:
                greyscale_feedrate = str(self.get_params(element[0],'F')/self.Fmax[1])
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
                        colors.append(greyscale_feedrate)
                        self.set_extremes(x,'Xmax')
                        self.set_extremes(y,'Ymax')

        print(len(x_coordinates))
        print(self.number_of_lines)
        print(self.Zmax)

        fig = plt.figure()
        ax = fig.add_subplot(111,projection='3d')

        ax.set_xlim(self.Xmax[0],self.Xmax[1])
        ax.set_ylim(self.Ymax[0],self.Ymax[1])
        largest_axis = max(self.Xmax[1]-self.Xmax[0],self.Ymax[1]-self.Ymax[0],self.Zmax[1]-self.Zmax[0])
        ax.set_box_aspect(((self.Xmax[1]-self.Xmax[0])/largest_axis,(self.Ymax[1]-self.Ymax[0])/largest_axis,(self.Zmax[1]-self.Zmax[0])/largest_axis))

        ax.plot(x_coordinates,y_coordinates,z_coordinates,label='Visualized gcode model',linewidth=0.1,color='b')
        ax.legend()
        plt.show()

        return 0

    def visualize_gcode_plotly(self):
        layer_height = 0
        extrusion_volume = 0
        current_feedrate = 0
        x_coordinates = []
        y_coordinates = []
        z_coordinates = []
        colors = []

        for element in self.list_of_intervals:
            if self.get_params(element[0],'F') >= 0:
                greyscale_feedrate = self.get_params(element[0],'F')/self.Fmax[1]
                colr = math.floor(self.get_params(element[0],'F')*255/self.Fmax[1])
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
                        colors.append(greyscale_feedrate)
                        self.set_extremes(x,'Xmax')
                        self.set_extremes(y,'Ymax')
                x_coordinates.append(None)
                y_coordinates.append(None)
                z_coordinates.append(None)
                colors.append(0)

        df = pd.DataFrame(dict(
            x = x_coordinates,
            y = y_coordinates,
            z = z_coordinates,
            colors = colors
        ))

        #fig = px.line_3d(df,x='x',y='y',z='z',color='colors')
        fig = go.Figure(data=go.Scatter3d(
            x=x_coordinates,y=y_coordinates,z=z_coordinates,
            mode="lines",
            line=dict(
                width=6,
                color=colors,
                colorscale=[[0,'rgb(0,0,255)'],[1,'rgb(255,0,0)']]
                ),
            connectgaps=False
            ))

        fig.show()


if __name__ == '__main__':
    robot = {}
    extruder_tool = ExtruderTool.ExtruderTool('FDM','10.0.0.3')
    filename = 'Circle'

    executioner = GCodeExecutor(filename,robot,extruder_tool)
    executioner.load_gcode()

    executioner.display()
    executioner.visualize_gcode()
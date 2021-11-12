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

        # initial values that should never be used before finding new values anyway
        self.Z = 50
        self.E = -100
        self.F = -100
        self.move_type = 'idle'

        self.robot = _robot
        self.extruder_tool = _extruder_tool


    def get_interval(self):
        return self.interval
    

    # Set the current interval of gcode lines
    def set_interval(self,interval):
        self.interval = interval


    def append_interval(self):
        self.list_of_intervals.append(self.interval)


    def read_command(self,line_number):
        return self.gcodelines[line_number].command[0] + str(self.gcodelines[line_number].command[1])


    def get_command(self):
        return self.command


    def set_command(self):
        interval = self.interval
        command = self.read_command(interval[0])
        self.command = command


    # read given param from gcode
    def read_param(self,line_number,param):
        try:
            return self.gcodelines[line_number].params[param]
        except:
            return False


    # get the current param from self
    def get_param(self,param):
        return self.__dict__[param]


    # set the current param to self
    def set_param(self,line_number,param):
        if self.read_param(line_number,param) != False:
            self.__dict__[param] = self.read_param(line_number,param)


    def set_extremes(self,param,extreme):
        try:
            if param > self.__dict__[extreme][1]:
                self.__dict__[extreme][1] = param
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
        
        ''' maybe change to while(if(if(break)),if(if(brake)),if(if(brake)),line += 1) '''

        line_number = self.interval[1]+1
        interval = [line_number,line_number]
        while (line_number < self.number_of_lines):
            if self.read_param(line_number,'Z') != False and self.read_param(line_number, 'Z') != self.get_param('Z'):
                interval = [self.interval[1]+1,line_number-1]
                self.set_param(line_number,'Z')
                #print("interval end due to layer height change")
                break
            elif self.read_param(line_number,'E') != False and self.read_param(line_number,'E') < self.get_param('E'):
                interval = [self.interval[1]+1,line_number-1]
                self.set_param(line_number,'E')
                #print("interval end due to E param change")
                break
            elif self.read_command(line_number) != self.read_command(self.interval[1]+1):
                interval = [self.interval[1]+1,line_number-1]
                #print("interval end due to command change")
                break
            elif self.read_param(line_number,'F') != False and self.read_param(line_number,'F') != self.get_param('F'): 
            # Check if this is wanted, or append this last x-y line to interval
                self.set_param(line_number,'F')
                self.set_extremes(self.F,'Fmax')
                if self.read_param(line_number,'X') == False and self.read_param(line_number,'Y') == False:
                    interval = [self.interval[1]+1,line_number-1]
                    #print("interval end due to F param change")
                    break
            else:
                # Typically when the next line is just another X-Y coordinate
                line_number = line_number + 1

        self.set_interval(interval)
        self.append_interval()

    def find_intervals(self):
        count = 0
        while self.number_of_lines > self.interval[1] + 1 and count < 100000:
            self.find_next_interval()
            count = count + 1

        self.current_interval = 1


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


    def make_waypoints(self,interval):
        # Take an interval of movement commands
        # Calculate trapesoidal movement/velocity to reduce jerk and uphold correct extrusion speed
        # ?
        num_waypoints = interval[1]+1-interval[0]
        xyz_coordinates = np.empty([num_waypoints,3])

        for point in range(interval[0],interval[1]+1):
            xyz_coordinates[[point][0]] = self.read_param(point,'X')
            xyz_coordinates[[point][1]] = self.read_param(point,'Y')
            xyz_coordinates[[point][2]] = self.Z
        return 0


    # Blocking action
    def run_code_segment(self,interval):
        command = self.read_command(interval[0])
        if command[0] == 'M':
            print("send to extruder_tool")
        if self.read_param(interval[0]) >= 0:
            self.Z = self.read_param(interval[0])
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
            if self.read_param(element[0],'F') != False:
                greyscale_feedrate = self.read_param(element[0],'F')/self.Fmax[1]
            if self.read_param(element[0],'Z') != False:
                layer_height = self.read_param(element[0],'Z')
                self.set_extremes(layer_height,'Zmax')
            if self.read_param(element[0],'E') != False:
                extrusion_volume = self.read_param(element[0],'E')
            if self.read_command(element[0]) == 'G1' and element[0] != element[1]:
                for point in range(element[0],element[1]+1):
                    if self.read_param(point,'E') > extrusion_volume and self.read_param(point,'E') != False:
                        x = self.read_param(point,'X')
                        y = self.read_param(point,'Y')
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

        fig = go.Figure(data=go.Scatter3d(
            x=x_coordinates,y=y_coordinates,z=z_coordinates,
            mode="lines",
            line=dict(
                width=6,
                color=colors,
                colorscale=[[0,'rgb(0,0,0)'],[1,'rgb(0,255,255)']]
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
import os
import time
import math
import sys

import argparse
import numpy as np
import pandas as pd
import plotly.graph_objects as go

from gcodeparser import GcodeParser
if sys.platform == 'linux':
    from frankx import Affine, LinearMotion, LinearRelativeMotion, Measure, MotionData, Reaction, Robot, RobotMode, RobotState, StopMotion, Waypoint, WaypointMotion
elif sys.platform == 'win32':
    try:
        from frankx import Affine, LinearMotion, LinearRelativeMotion, Measure, MotionData, Reaction, Robot, RobotMode, RobotState, StopMotion, Waypoint, WaypointMotion   
    except Exception as e:
        print(e)
    finally:
        print('Running on OS: ' + sys.platform)

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

        # default planar bed
        self.bed_plane_abcd = [0,0,0,0]


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
        if self.list_of_intervals == []:
            self.append_interval()

        if self.interval[1]+1 >= self.number_of_lines:
            print("End of code")
            return 'End of code'

        ''' maybe change to while(if(if(break)),if(if(brake)),if(if(brake)),line += 1) '''

        line_number = self.interval[1]+1
        interval = [line_number,line_number]
        while (line_number < self.number_of_lines):

            # Find dimensions of model
            if self.read_param(line_number,'X') != False:
                self.set_extremes(self.read_param(line_number,'X'),'Xmax')
            if self.read_param(line_number,'Y') != False:
                self.set_extremes(self.read_param(line_number,'Y'),'Ymax')
            if self.read_param(line_number,'Z') != False:
                self.set_extremes(self.read_param(line_number,'Z'),'Zmax')

            # Find desired interval change condition
            if self.read_param(line_number,'Z') != False and self.read_param(line_number, 'Z') != self.get_param('Z'):
                interval = [self.interval[1]+1,line_number-1]
                if interval[1]<interval[0]:
                    interval = [line_number,line_number]
                self.set_param(line_number,'Z')
                break
            elif self.read_param(line_number,'E') != False and self.read_param(line_number,'E') < self.get_param('E'):
                interval = [self.interval[1]+1,line_number-1]
                if interval[1]<interval[0]:
                    interval = [line_number,line_number]
                self.set_param(line_number,'E')
                break
            elif self.read_command(line_number) != self.read_command(self.interval[1]+1):
                interval = [self.interval[1]+1,line_number-1]
                if interval[1]<interval[0]:
                    interval = [line_number,line_number]
                break
            elif self.read_param(line_number,'F') != False and self.read_param(line_number,'F') != self.get_param('F'): 
            # Check if this is wanted, or append this last x-y line to interval
                self.set_param(line_number,'F')
                self.set_extremes(self.F,'Fmax')
                if self.read_param(line_number,'X') == False and self.read_param(line_number,'Y') == False:
                    interval = [self.interval[1]+1,line_number-1]
                    if interval[1]<interval[0]:
                        interval = [line_number,line_number]
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

        # gcode is in mm, change to m for robot
        for line in self.gcodelines:
            for element in line.params:
                if element == 'X':
                    line.update_param('X',line.get_param('X')/1000)
                if element == 'Y':
                    line.update_param('Y',line.get_param('Y')/1000)
                if element == 'Z':
                    line.update_param('Z',line.get_param('Z')/1000)

        self.number_of_lines = len(self.gcodelines)
        self.find_intervals()

    def convert_per_minute_to_per_second(self,value_per_minute):
        return value_per_minute/60

    def convert_per_second_to_per_minute(self,value_per_second):
        return value_per_second*60

    def calculate_difference(self,first_value,second_value):
        return second_value-first_value

    def calculate_delta_t(self,feedrate,delta_mm):
        return delta_mm/self.convert_per_minute_to_per_second(feedrate)

    def calculate_max_velocity(self,feedrate):
        cartesian_max_vel = 1700 #mm/s
        return self.convert_per_minute_to_per_second(feedrate)/cartesian_max_vel


    def home_gcode(self,homing_type):
        if homing_type == 'Guiding':
            self.robot.robot_home_move()

            print("INFO: To enter guiding mode, EMERGENCY STOP button needs to be pressed DOWN and the robot light should be continuously WHITE!")
            print("After positioning robot, open the EMERGENCY STOP button to its UP position! The robot should the have BLUE lights")
            input("Position end-effector nozzle < 1cm from desired (0,0) location.\nWhen satisfied with position, press Enter to continue...")

            self.gcode_home_pose = self.robot.read_current_pose()
            self.gcode_home_pose_vec = self.gcode_home_pose.vector()

            # Can potentially add collision detection here to further improve home point. 
            # But new (0,0) point can be chosen from mid point of bed probing afterwards

            #self.robot.set_robot_mode('Idle')

        elif homing_type == 'known':
            print("Set gcode_home to this value. Home point assumed known")
            self.gcode_home_pose_vec = [0.48,0.0,0.0,math.pi/2,0.0,0.0]

        else:
            print("Failed to home gcode zero... Check RobotMode input")

    def probe_bed(self):
        probe_locations_xy = [[[0.1,0.1],[0,0.1],[-0.1,0.1]],[[0.1,0],[0,0],[-0.1,0]],[[0.1,-0.1],[0,-0.1],[-0.1,-0.1]]] # relative to ghome_gcode

        # Apply reaction motion if the force in negative z-direction is greater than 10N
        reaction_motion = LinearRelativeMotion(Affine(0.0, 0.0, 0.01))  # Move up for 1cm

        bed_grid = [[[],[],[]],[[],[],[]],[[],[],[]]] # ugly ik

        contact_found = True

        for axis1 in range(3):
            for axis2 in range(3):

                self.robot.robot.set_dynamic_rel(0.1)
                # Move to probe location
                m1 = LinearMotion(Affine(probe_locations_xy[axis1][axis2][0] + self.gcode_home_pose_vec[0],probe_locations_xy[axis1][axis2][1] + self.gcode_home_pose_vec[1],self.gcode_home_pose_vec[2]))
                self.robot.robot.move(m1)

                # Reset data reaction motion
                d2 = MotionData().with_reaction(Reaction(Measure.ForceZ < -5.0, reaction_motion))

                self.robot.robot.set_dynamic_rel(0.02)

                # Move slowly towards print bed
                m2 = LinearRelativeMotion(Affine(0.0,0.0,-0.05))
                self.robot.robot.move(m2, d2)

                # Check if the reaction was triggered
                if d2.did_break:
                    self.robot.robot.recover_from_errors()
                    print('Force exceeded 5N!')
                    print(f"Hit something for probe location x: {axis1}, y: {axis2}")

                    current_pose = self.robot.read_current_pose()
                    vector_pose = current_pose.vector()
                    probe_point = vector_pose[0:3]
                    probe_point[2] = probe_point[2] - 0.01 # correcting for moving 1 cm up after contact
                    bed_grid[axis1][axis2] = probe_point

                elif not d2.did_break:
                    print(f"Did not hit anything for probe location x: {axis1}, y: {axis2}")
                    contact_found = False

        if contact_found:
            self.bed_points = bed_grid
            self.calculate_bed_surface_plane()
            self.gcode_home_pose_vec = bed_grid[1][1]
        else:
            print("One or more bed points was not found")

    def calculate_bed_surface_plane(self):
        '''
        Calculate the plane equation given by ax + by + cz + d = 0
        
        Usage
        -----
        Uses 3 non-collinear points A, B and C and calculates the cross product AB x AC = [a,b,c] and d = -(aAx + bAy + cAz)
        '''
        # Points
        A = self.bed_points[1][1]
        B = self.bed_points[2][0]
        C = self.bed_points[0][2]

        AB = [B[0]-A[0],B[1]-A[1],B[2]-A[2]]
        AC = [C[0]-A[0],C[1]-A[1],C[2]-A[2]]

        ABxAC = np.cross(AB,AC)
        a = ABxAC[0]
        b = ABxAC[1]
        c = ABxAC[2]
        d = -(a*A[0] + b*A[1] + c*A[2])
        self.bed_plane_abcd = [a,b,c,d]

    def vertical_bed_level_compensation(self,point):
        '''
        Calculates the vertical compensation needed due to non-horizontal build plane.
        Compensation is made based on z height equal to 0, making the z component of 'point' excessive and unused
        '''
        return -(self.bed_plane_abcd[0]*point[0] + self.bed_plane_abcd[1]*point[1] + self.bed_plane_abcd[3])

    def does_model_fit_bed(self):
        # Assumes a Cubic build volume.. Should use actual spherical volume
        # Change home pose to gcode home, checks then the spedific placement after homeing gcode zero
        if ((self.Xmax[1]+self.robot.gcode_home_pose_vec[0] >= self.robot.radius) 
            or (self.Xmax[0]+self.robot.gcode_home_pose_vec[0] <= self.robot.base_area) 
            or (self.Ymax[1]+self.robot.gcode_home_pose_vec[1] >= self.robot.radius) 
            #or (self.Ymax[0]+self.robot.gcode_home_pose_vec[1] <= self.robot.base_area) 
            or (self.Zmax[1]+self.robot.gcode_home_pose_vec[2] >= self.robot.height_up) 
            or (self.Zmax[0]+self.robot.gcode_home_pose_vec[2] <= self.robot.height_down)):
            return False
        else:
            return True

    def is_build_feasible(self):
        if not self.does_model_fit_bed():
            print("Model does not fit on build plate")
            return False
        else: # skipping lower part for now
            return True

        # move in a square and detect collision
        edge_points = [[self.Xmax[1],self.Ymax[1],self.Zmax[0]+0.02],[self.Xmax[0],self.Ymax[1],self.Zmax[0]+0.02],[self.Xmax[0],self.Ymax[0],self.Zmax[0]+0.02],[self.Xmax[1],self.Ymax[0],self.Zmax[0]+0.02],[self.Xmax[1],self.Ymax[1],self.Zmax[0]+0.02]]
        reaction_motion = LinearMotion(Affine(self.robot.robot_home_pose_vec[0],self.robot.robot_home_pose_vec[1],self.robot.robot_home_pose_vec[2]))

        for point in edge_points:

            # Stop motion if the overall force is greater than 30N
            data = MotionData().with_reaction(Reaction(Measure.ForceXYZNorm > 30.0, reaction_motion))
            motion = LinearMotion(Affine(point[0] + self.robot.robot_home_pose_vec[0],point[1] + self.robot.robot_home_pose_vec[1],point[2]))

            self.robot.robot.move(motion,data)

            if data.did_break:
                self.robot.robot.recover_from_errors()
                print("Collision when checking build area.")
                return False

            reaction_motion = LinearMotion(Affine(point[0] + self.robot.robot_home_pose_vec[0],point[1] + self.robot.robot_home_pose_vec[1],point[2]))

        self.is_area_clear = True

        return True

    def make_waypoints(self,interval):
        # Take an interval of movement commands
        # Calculate trapesoidal movement/velocity to reduce jerk and uphold correct extrusion speed
        # ?
        num_waypoints = interval[1]+1-interval[0]
        waypoints = []

        for point in range(interval[0],interval[1]+1):
            waypoints.append(Waypoint(Affine(self.read_param(point,'X') + self.gcode_home_pose_vec[0],self.read_param(point,'Y') + self.gcode_home_pose_vec[1],self.Z + self.gcode_home_pose_vec[2])))
        return waypoints


    # Blocking action
    def run_code_segment(self,interval):
        command = self.read_command(interval[0])

        if command[0] == 'M':
            print("send to extruder_tool")
            return 0

        # Reset extruder distance
        if self.read_param(interval[0],'E') != False and command == 'G92':
            self.E = self.read_param(interval[0],'E')

        # Find current / new z-height
        if self.read_param(interval[0],'Z') != False:
            self.Z = self.read_param(interval[0],'Z')

        # Find desired feedrate / working speed
        if self.read_param(interval[0],'F') != False:
            self.F = self.read_param(interval[0],'F')

        if command == 'G0':
            # Stop extrusion and move to target
            if self.read_param(interval[0],'X') != False and self.read_param(interval[0],'Y') != False:
                print([self.read_param(interval[0],'X') + self.gcode_home_pose_vec[0],self.read_param(interval[0],'Y') + self.gcode_home_pose_vec[1],self.Z + self.gcode_home_pose_vec[2]])
                input("continue? lin move...")
                self.robot.lin_move_to_point(self.read_param(interval[0],'X') + self.gcode_home_pose_vec[0],self.read_param(interval[0],'Y') + self.gcode_home_pose_vec[1],self.Z + self.gcode_home_pose_vec[2])

        elif command == 'G1':
            # Find waypoints, trapesoidal movement?
            final_pose = [self.read_param(interval[1],'X') + self.gcode_home_pose_vec[0],self.read_param(interval[1],'Y') + self.gcode_home_pose_vec[1],self.Z + self.gcode_home_pose_vec[2]]

            if interval[1]+1-interval[0] > 2: # arbitrary minimum waypoints, Remember to change! (this is going to go well....)
                waypoints = self.make_waypoints(interval)
                # check bounds?
                # set extrusion speed
                # feed waypoints to robot and listen to robot pose
                input("start waypoint move...")

                self.robot.robot.set_dynamic_rel(0.05)
                motion = WaypointMotion(waypoints,retrun_when_finished=False)
                thread = self.robot.robot.move_async(motion)

                print(self.robot.read_current_pose())
                print(final_pose)

                while self.robot.read_current_pose() != final_pose:
                    time.sleep(1)
                    print("final pose: ")
                    print(final_pose)
                    print("Current pose: ")
                    print(self.robot.read_current_pose())

            if self.read_param(interval[0],'E') != False:
                self.E = self.read_param(interval[0],'E')

    def visualize_gcode(self):
        z = 0
        extrusion_distance = 0
        current_feedrate = 0
        x_coordinates = []
        y_coordinates = []
        z_coordinates = []
        colors = []

        for element in self.list_of_intervals:
            if self.read_param(element[0],'E') != False:
                extrusion_distance = self.read_param(element[0],'E')

            if self.read_param(element[0],'Z') != False:
                z = self.read_param(element[0],'Z')
                self.Z = z

            if self.read_param(element[0],'F') != False:
                greyscale_feedrate = self.read_param(element[0],'F')#/self.Fmax[1]

            if self.read_command(element[0]) == 'G1':
                try: # front-pad the start position to the coming series of moves
                    x_coordinates.append(self.X * 1000)
                    y_coordinates.append(self.Y * 1000)
                    z_coordinates.append(self.Z * 1000)
                    colors.append(greyscale_feedrate)
                except: # For initial G1 commands before any x-y coodrinates have been set
                    pass

                for point in range(element[0],element[1]+1):
                    if self.read_param(point,'X') != False and self.read_param(point,'Y') != False:
                        x = self.read_param(point,'X')
                        y = self.read_param(point,'Y')

                        self.X = x
                        self.Y = y

                        if self.read_param(point,'E') > extrusion_distance and self.read_param(point,'E') != False:
                            x_coordinates.append(x * 1000)
                            y_coordinates.append(y * 1000)
                            z_coordinates.append(z * 1000)
                            colors.append(greyscale_feedrate)

                            extrusion_distance = self.read_param(point,'E')

                x_coordinates.append(None)
                y_coordinates.append(None)
                z_coordinates.append(None)
                colors.append(0)

            elif self.read_command(element[0]) == 'G0':
                for point in range(element[0],element[1]+1):
                    if self.read_param(point,'X') != False and self.read_param(point,'Y') != False:
                        x = self.read_param(point,'X')
                        y = self.read_param(point,'Y')

                        self.X = x
                        self.Y = y


        largest_axis = max([self.Xmax[1]-self.Xmax[0],self.Ymax[1]-self.Ymax[0],self.Zmax[1]-self.Zmax[0]])
        axis_scale = [(self.Xmax[1]-self.Xmax[0])/largest_axis,(self.Ymax[1]-self.Ymax[0])/largest_axis,(self.Zmax[1]-self.Zmax[0])/largest_axis]

        fig = go.Figure(data=go.Scatter3d(
            x=x_coordinates,y=y_coordinates,z=z_coordinates,
            mode="lines",
            line=dict(
                width=6,
                color=colors,
                cmin=0,
                cmax=self.Fmax[1],
                colorbar=dict(
                    borderwidth=0,
                    title=dict(
                        text='Feedrate [mm/min]'
                    )
                ),
                colorscale=[[0,'rgb(255,0,0)'],[1,'rgb(0,0,255)']]
                ),
            connectgaps=False
            ))

        fig.update_layout(
            autosize=True,
            scene=dict(
                aspectratio = dict( x=axis_scale[0], y=axis_scale[1], z=axis_scale[2] ),
                aspectmode = 'manual'
            ),
            title_text=('Visualization of extruded filament paths for ' + self.filename),
            showlegend=False
        )

        fig.show()


    def __str__(self):
        return "gcodeexecutor"


    def display(self):
        if '.gcode' in self.filename:
            print(f"\nName of file being processed: {self.filename}")
        else:
            print(f"\nName of file being processed: {self.filename + '.gcode'}")
        print(f"Number of command lines processed: {self.number_of_lines}")
        print("Total filament used: \n")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(add_help=True)

    parser.add_argument('--gfile',default='Circle')

    args = parser.parse_args()

    model = GCodeExecutor(args.gfile,{},{})
    model.load_gcode()
    model.visualize_gcode()
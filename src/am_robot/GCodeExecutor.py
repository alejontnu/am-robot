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
    from frankx import Affine, LinearMotion, LinearRelativeMotion, Measure, MotionData, PathMotion, Reaction, Robot, RobotMode, RobotState, StopMotion, Waypoint, WaypointMotion
elif sys.platform == 'win32':
    try:
        from frankx import Affine, LinearMotion, LinearRelativeMotion, Measure, MotionData, PathMotion, Reaction, Robot, RobotMode, RobotState, StopMotion, Waypoint, WaypointMotion   
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
    def __init__(self,_filename,_robot,_tool):
        self.filename = _filename
        self.interval = [0,0] # On the assumption that the first and second gcode command will allways be unique from eachother
        self.list_of_intervals = []

        # initial values that should never be used before finding new values anyway
        self.X = 0
        self.Y = 0
        self.Z = 0
        self.E = 0
        self.F = 0
        self.move_type = 'idle'

        self.robot = _robot
        self.tool = _tool

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
            # if self.read_param(line_number,'Z') != False and self.read_param(line_number, 'Z') != self.get_param('Z'):
            #     interval = [self.interval[1]+1,line_number-1]
            #     if interval[1]<interval[0]:
            #         interval = [line_number,line_number]
            #     self.set_param(line_number,'Z')
            #     break
            if self.read_command(line_number) != self.read_command(self.interval[1]+1):
                interval = [self.interval[1]+1,line_number-1]
                break
            elif self.read_param(line_number,'E') != False and self.read_param(line_number,'E') < self.get_param('E'):
                interval = [self.interval[1]+1,line_number-1]
                self.set_param(line_number,'E')
                break
            elif self.read_param(line_number,'F') != False and self.read_param(line_number,'F') != self.get_param('F'): 
            # Check if this is wanted, or append this last x-y line to interval
                self.set_param(line_number,'F')
                self.set_extremes(self.F,'Fmax')
                nr_keys = 0
                for key in self.gcodelines[line_number].params:
                    nr_keys = nr_keys + 1
                    if nr_keys > 1:
                        interval = [self.interval[1]+1,line_number-1]
                        break
            else:
                # Typically when the next line is just another X-Y coordinate
                line_number = line_number + 1

        if interval[1] < interval[0]:
            interval = [line_number,line_number]

        self.set_interval(interval)
        self.append_interval()

    def find_intervals(self):
        while self.number_of_lines > self.interval[1] + 1:
            self.find_next_interval()
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

                # Reset data reaction motion, may need to tweek trigger force when extruder is mounted
                d2 = MotionData().with_reaction(Reaction(Measure.ForceZ < -2.0, reaction_motion))

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
            print(f"Gcode home location: {self.gcode_home_pose_vec}")
        else:
            print("One or more bed points was not found")

        return contact_found

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
        print(f"Bed plane coefficients: {self.bed_plane_abcd}")

    def vertical_bed_level_compensation(self,point):
        '''
        Calculates the vertical compensation needed due to non-horizontal build plane.
        Compensation is made based on z height equal to 0, making the z component of 'point' excessive and unused
        '''
        return -(self.bed_plane_abcd[0]*point[0] + self.bed_plane_abcd[1]*point[1] + self.bed_plane_abcd[3])

    def does_model_fit_bed(self):
        # Assumes a Cubic build volume.. Should use actual spherical volume
        # Change home pose to gcode home, checks then the spedific placement after homeing gcode zero
        if ((self.Xmax[1]+self.gcode_home_pose_vec[0] >= self.robot.radius) 
            or (self.Xmax[0]+self.gcode_home_pose_vec[0] <= self.robot.base_area) 
            or (self.Ymax[1]+self.gcode_home_pose_vec[1] >= self.robot.radius) 
            #or (self.Ymax[0]+self.gcode_home_pose_vec[1] <= self.robot.base_area) 
            or (self.Zmax[1]+self.gcode_home_pose_vec[2] >= self.robot.height_up) 
            or (self.Zmax[0]+self.gcode_home_pose_vec[2] <= self.robot.height_down)):
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
        waypoints = []
        for point in range(interval[0],interval[1]+1):
            z_compensation = self.vertical_bed_level_compensation((self.read_param(point,'X') + self.gcode_home_pose_vec[0],self.read_param(point,'Y') + self.gcode_home_pose_vec[1],self.Z + self.gcode_home_pose_vec[2]))
            waypoints.append(Waypoint(Affine(self.read_param(point,'X') + self.gcode_home_pose_vec[0],self.read_param(point,'Y') + self.gcode_home_pose_vec[1],self.Z + self.gcode_home_pose_vec[2] + z_compensation)))
        return waypoints

    def make_path(self,interval,corner_blend_threshold):
        path_points = []
        for point in range(interval[0],interval[1]+1):
            for key in self.gcodelines[point].params:
                if key == 'E':
                    pass
                else:
                    self.__dict__[key] = self.read_param(point,key)
            z_compensation = self.vertical_bed_level_compensation((self.X + self.gcode_home_pose_vec[0],self.Y + self.gcode_home_pose_vec[1],self.Z + self.gcode_home_pose_vec[2]))
            print(z_compensation)
            path_points.append(Affine(self.X + self.gcode_home_pose_vec[0],self.Y + self.gcode_home_pose_vec[1],self.Z + self.gcode_home_pose_vec[2] + z_compensation))
        path = PathMotion(path_points,blend_max_distance=corner_blend_threshold)
        return path

    def target_point(self,point):
        '''
        point is an index for desired gcodeline
        '''
        z_compensation = self.vertical_bed_level_compensation((self.read_param(point,'X') + self.gcode_home_pose_vec[0],self.read_param(point,'Y') + self.gcode_home_pose_vec[1],self.Z + self.gcode_home_pose_vec[2]))
        return self.read_param(point,'X') + self.gcode_home_pose_vec[0],self.read_param(point,'Y') + self.gcode_home_pose_vec[1],self.Z + self.gcode_home_pose_vec[2] + z_compensation


    # Blocking action
    def run_code_segment(self,interval):
        command = self.read_command(interval[0])
        print(command[0])
        # Handle different M (machine) commands
        if command[0] == 'M':
            print("send to tool")
            if command == 'M82':
                print("E absolute")
            elif command == 'M83':
                print("E Relative")
            elif command == 'M84':
                print("Disable motors")
            elif command == 'M104':
                print("Setting hotend temperature")
                self.tool.set_nozzletemp(self.read_param(interval[0],'S'))
            elif command == 'M105':
                print("Getting nozzle temperature reading")
                nozzle_temp = self.tool.read_temperature()
            elif command == 'M106':
                print("Set fan speed")
                self.tool.set_fanspeed(self.read_param(interval[0],'S'))
            elif command == 'M107':
                print("Fan off")
            elif command == 'M1095':
                print("Waiting for hotend temperature")
                if self.read_param(interval[0],'S') != False:
                    self.tool.set_nozzletemp(self.read_param(interval[0],'S'))
                    while self.tool.read_temperature() < self.read_param(interval[0],'S')-5:
                        print(".")
                        time.sleep(1)
                elif self.read_param(interval[0],'R') != False:
                    self.tool.set_nozzletemp(self.read_param(interval[0],'R'))
                    while self.tool.read_temperature() < self.read_param(interval[0],'R')-5 or self.tool.read_nozzletemp() > self.read_param(interval[0],'R')+5:
                        print(".")
                        time.sleep(1)
                else:
                    self.tool.set_nozzletemp(0)
                    while self.tool.read_temperature() > 30: #assumed high ambient temperature
                        print(".")
                        time.sleep(1)
                
            elif command == 'M140':
                print("Set bed temperature")
            else:
                print(f"No action for command: {command}")

        elif command[0] == 'G':

            # Find current / new z-height %% Implement for each line instead of start of interval, ignored if no new value anyway %%
            if self.read_param(interval[0],'Z') != False:
                self.Z = self.read_param(interval[0],'Z')

            # Find desired feedrate / working speed / max velocity
            if self.read_param(interval[0],'F') != False:
                self.F = self.read_param(interval[0],'F')

            if command == 'G0':
                # Stop extrusion and move to target
                self.tool.set_feedrate(0) # just incase
                if self.read_param(interval[0],'X') != False or self.read_param(interval[0],'Y') != False or self.read_param(interval[0],'Z') != False:
                    motion = self.make_path(interval,0.01)
                    self.robot.robot.velocity_rel = self.tool.calculate_max_rel_velocity(self.F,self.robot.max_cart_vel)

                    input("Press enter to start non-extrusion move...")
                    thread = self.robot.robot.move_async(motion)
                    thread.join()


            elif command == 'G1':
                if (self.read_param(interval[0],'X') != False) or (self.read_param(interval[0],'Y') != False) or (self.read_param(interval[0],'Z') != False):
                    # Make path trajectory

                    print(interval)

                    motion = self.make_path(interval,0.002) # PathMotion gives smooth movement compared to WayPointMovement

                    # set dynamic rel and relative max velocity based on feedrate
                    self.robot.robot.velocity_rel = self.tool.calculate_max_rel_velocity(self.F,self.robot.max_cart_vel)

                    # Accept check
                    input("Press enter to start extrusion move move...")

                    # set extrusion speed if needed. Some slicers use G1 for non extrusion moves...
                    if self.read_param(interval[1],'E') != False:
                        self.tool.set_feedrate(self.F)

                    # feed path motion to robot and move using a separate thread
                    thread = self.robot.robot.move_async(motion) # Just starts move in a thread with some initialization

                    # testing stuff
                    # print(type(RobotState))
                    # print(RobotState)
                    # print(RobotState("robot_mode"))
                    # #print(self.robot.read_current_pose())
                    # #print(final_pose)
                    # print(thread)

                    # for i in range(3):
                    #     print(i)
                    #     #print(self.robot.robot.current_pose()) # This gives an error when robot is threaded...
                    #     #Configure extruder here based on robot dynamics...
                    #     #Is what i would have done if robot state was available...
                    #     input("Enter to continue...")

                    print("waiting on thread to finish motion")
                    # Wait here for path motion to finish and join the thread
                    thread.join()

                    # Thread done aka move done aka stop extrusion immidiately
                    self.tool.set_feedrate(0)

                elif self.read_param(interval[0],'E') != False:
                    # Target extrusion distance and time elapsed at given feedrate
                    target_E = self.read_param(interval[0],'E')
                    sleep_time = self.tool.calculate_delta_t(target_E,self.E,self.F)

                    # set retraction/un-retraction feedrate
                    self.tool.set_feedrate(np.sign(sleep_time)*self.F)
                    # Sleep
                    time.sleep(abs(sleep_time))
                    # Stop retraction/un-retraction
                    self.tool.set_feedrate(0)

                if self.read_param(interval[1],'E') != False:
                    self.E = self.read_param(interval[0],'E')

            elif command == 'G21':
                print("set units to millimeters")
            elif command == 'G28':
                print("Auto home")
            elif command == 'G90':
                print("use absolute coordinates")

            # Reset extruder/all distances
            elif command == 'G92':
                for key in self.gcodelines[interval[0]].params:
                    self.__dict__[key] = self.read_param(interval[0],key)

        else:
            print(f"No action for command: {command}")

    def visualize_bed_mesh(self):
        # add plotly of bed mesh here
        return 0

    def visualize_gcode(self):
        z = 0
        extrusion_distance = 0
        current_feedrate = 0
        x_coordinates = []
        y_coordinates = []
        z_coordinates = []
        colors = []

        print("May have issues with 1 million+ points...")

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

                        if self.read_param(point,'Z') != False:
                            z = self.read_param(point,'Z')

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

                        if self.read_param(point,'Z') != False:
                            z = self.read_param(point,'Z')


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

        self.reset_parameters()

    def reset_parameters(self):
        # corresponding to 0,0,0 of start of printing
        self.X = 0
        self.Y = 0
        self.Z = 0
        self.F = 0
        self.E = 0

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
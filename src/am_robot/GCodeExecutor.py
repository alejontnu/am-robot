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
from am_robot.GCodeCommands import GCodeCommands

class GCodeExecutor(GCodeCommands):
    '''
    Read and parse gcode into a full object variable

    Attributes:
    -----------
    filename: string
        filename to look for when loading G-code
    robot: Class
        Class object of the robot used
    tool: Class
        Class object of the tool head used

    Methods:
    -----------
    load_gcode():
        finds and runs pre-processing of G-code given by the attribute 'filename'
    run_code_segment():
        Used to run a single interval of G-code, such as a motion trajectory or machine setting change
    probe_bed():
        Used to run the bed probing sequence for determining bed flatness
    display():
        Displays basic information about G-code file
    visualize_gcode():
        Visualizes motion trajectories where extrusion is done, plotting a 3D line plot of the G-code model
    visualize_bed_mesh():
        Plots the bed mesh generated from the bed probing sequence

    '''
    def __init__(self,filename,robot,tool):
        '''
        Initializes the Class object

        Input:
        -----
        filename: string
            filename to look for when loading G-code
        robot: Class
            Class object of the robot used
        tool: Class
            Class object of the tool head used

        Returns:
        -----
        Initialized Class object

        '''
        super().__init__()

        self.filename_ = filename
        self.interval = [0,0] # On the assumption that the first and second gcode command will allways be unique from eachother. This is a valid assumption
        self.list_of_intervals = []

        # initial values that should never be used before finding new values anyway
        self.prev_X = 0
        self.prev_Y = 0
        self.prev_Z = 0
        self.X = 0
        self.Y = 0
        self.Z = 0
        self.E = 0
        self.F = 0
        self.move_type = 'idle'

        self.robot = robot
        self.tool = tool

        # default planar bed
        self.bed_plane_abcd = [0,0,0,0]


    def get_interval(self):
        '''
        Returns the current G-code interval

        Input:
        -----

        Returns:
        -----
        self.interval: [int,int]
            list of index of start and end line of interval in G-code
        '''
        return self.interval


    # Set the current interval of gcode lines
    def set_interval(self,interval):
        '''
        Sets the G-code interval

        Input:
        -----
        interval: [int,int]
            Sets the interval of indexes for the start and end line of G-code interval

        Returns:
        -----

        '''
        self.interval = interval


    def append_interval(self):
        '''
        Appends current self.interval to a list of intervals

        Input:
        -----

        Returns:
        -----

        '''
        self.list_of_intervals.append(self.interval)


    def read_command(self,line_number):
        '''
        Returns the command for the given G-code line number

        Input:
        -----
        line_number: int
            G-code line index for reading command

        Returns:
        -----
        command: string
            Command for the specific line as 'letter+number' i.e. 'G28'

        '''
        return self.gcodelines[line_number].command[0] + str(self.gcodelines[line_number].command[1])


    def get_command(self):
        '''
        Returns the current command

        Input:
        -----

        Returns:
        -----
        command: string
            Command as a string i.e. 'G28'

        '''
        return self.command


    def set_command(self):
        '''
        Sets the first command of the current interval

        Input:
        -----

        Returns:
        -----

        '''
        interval = self.interval
        command = self.read_command(interval[0])
        self.command = command


    # read given param from gcode
    def read_param(self,line_number,param):
        '''
        Reads the parameter specified by 'param' for the given G-code line. If no such param exists, returns False

        Input:
        -----
        line_number: int
            G-code line index for reading parameter
        param: string
            String type of parameter to read, i.e. 'X' or 'F'

        Returns:
        -----
        param_value: float
            Float value of parameter

        '''
        try:
            return self.gcodelines[line_number].params[param]
        except:
            return False


    # get the current param from self
    def get_param(self,param):
        '''
        Gets the current value of the parameter

        Input:
        -----
        param: string
            String type of parameter to return, i.e. 'X' or 'F'

        Returns:
        -----
        param_value: float
            Float value of parameter

        '''
        return self.__dict__[param]


    # set the current param to self
    def set_param(self,line_number,param):
        '''
        Sets the parameter value from the given G-code line to the object

        Input:
        -----
        line_number: int
            G-code line index for reading parameter
        param: string
            String type of parameter to set, i.e. 'X' or 'F'

        Returns:
        -----

        '''
        if self.read_param(line_number,param) != False:
            self.__dict__[param] = self.read_param(line_number,param)


    def set_extremes(self,param,extreme):
        '''
        Update the extremity values of given parameter

        Input:
        -----
        param: string
            String type of parameter to update, i.e. 'X' or 'F'
        extreme: float
            Value of paremeter to update with

        Returns:
        -----

        '''
        try:
            if param > self.__dict__[extreme][1]:
                self.__dict__[extreme][1] = param
            if param < self.__dict__[extreme][0]:
                self.__dict__[extreme][0] = param
        except:
            self.__dict__[extreme] = [param,param]

    def set_params(self,line_number):
        '''
        Sets all present parameters for the given G-code line

        Input:
        -----
        line_number: int
            G-code line index to read parameters from

        Returns:
        -----

        '''
        for key in self.gcodelines[line_number].params:
            self.__dict__[key] = self.read_param(line_number,key)

    def set_prev_xyz(self):
        self.prev_X = self.X
        self.prev_Y = self.Y
        self.prev_Z = self.Z


    # Find the next instance of retraction (NB may be 0mm retraction)
    def find_next_interval(self):
        '''
        Finds the next interval bounded by certain criteria from when an interval should end and adds the interval to the list of intervals. Critria include change in material extrusion direction, command type change, layer change, change in process speed, etc..

        Input:
        -----

        Returns:
        -----
        'End of code': string
            String indicating end of G-code has been reached

        '''
        '''
        priority:
        change on command change
        change on retraction/reversing of material/tool feedrate

        
        '''
        if self.list_of_intervals == []:
            self.append_interval()

        if self.interval[1]+1 >= self.number_of_lines:
            print("End of code")
            return 'End of code'

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
            if self.read_param(line_number,'F') != False:
                self.set_extremes(self.read_param(line_number,'F'),'Fmax')

            # Find desired interval change condition
            # Check if next line has a command has changed
            if self.read_command(line_number) != self.read_command(self.interval[1]+1):
                interval = [self.interval[1]+1,line_number-1]
                break

            # Check reversing of material extrusion
            elif self.read_param(line_number,'E') != False and self.read_param(line_number,'E') < self.get_param('E'):
                interval = [self.interval[1]+1,line_number-1]
                break

            elif self.read_param(line_number,'F') != False and self.read_param(line_number,'F') != self.get_param('F'): 
                interval = [self.interval[1]+1,line_number-1]
                break

            elif self.read_param(line_number-1,'E') != False and self.read_param(line_number,'E') == False and self.read_command(line_number-1) == 'G1':
                interval = [self.interval[1]+1,line_number-1]
                break

            elif (line_number > self.interval[1]+1) and self.turn_angle(line_number) > math.pi/4.0: # An overall turning radius would maybe be better
                interval = [self.interval[1]+1,line_number-1]
                break

            else:
                # Typically when the next line is just another X-Y coordinate
                self.set_prev_xyz()
                self.set_params(line_number)
                line_number = line_number + 1

        if interval[1] < interval[0]:
            interval = [line_number,line_number]
            self.set_prev_xyz()
            self.set_params(line_number)

        self.set_interval(interval)
        self.append_interval()

    def find_intervals(self):
        '''
        Calls find_next_interval() repeatedly until all lines have been added to an interval

        Input:
        -----

        Returns:
        -----

        '''
        while self.number_of_lines > self.interval[1] + 1:
            self.find_next_interval()
        self.reset_parameters()


    def load_gcode(self,lines):
        '''
        Finds the 'filename' file with '.gcode' extension in the data folder and parses the file using GcodeParser. X, Y, Z parameters are converted to the size used by the robot, i.e. G-code millimeter is changed to robot meter. Calls find_intervals() to starts pre-processing of G-code.

        Input:
        -----

        Returns:
        -----

        '''
        # Add .gcode if not given
        filename, extension = os.path.splitext(self.filename_)
        if extension == '':
            print("No file extension given, assuming '.gcode'")
            extension = '.gcode'
        elif extension.lower() == '.gcode':
            filename, extension = os.path.splitext(self.filename_)
        else:
            print("File extension not empty or .gcode")
            input("finding for '.gcode' instead. Enter to continue...")
            extension = '.gcode'

        cwd = os.getcwd()

        for root, dirs, files in os.walk(cwd):
            if filename+extension in files:
                fullpath = os.path.join(root, filename+extension)

        # Read file
        with open(fullpath,'r') as file:
            gcodedata = file.read()

        # Change instances of "-." to "-0.". Was causing issues with visualization plot
        gcodedata = gcodedata.replace('-.','-0.')

        # Parse lines into Dict object
        self.gcodelines = GcodeParser(gcodedata).lines

        # gcode is in mm, change to m for robot
        for line in self.gcodelines:
            for element in line.params:
                if element == 'X':
                    line.update_param('X',line.get_param('X')/1000.0)
                if element == 'Y':
                    line.update_param('Y',line.get_param('Y')/1000.0)
                if element == 'Z':
                    line.update_param('Z',line.get_param('Z')/1000.0)

        self.number_of_lines = len(self.gcodelines)
        if self.number_of_lines > lines:
            self.number_of_lines = lines
        self.find_intervals()


    def home_gcode(self,homing_type):
        '''
        Pauses to let the operator manually position the nozzle a small distance from the desired zero point of the G-code. A known point can also be used

        Input:
        -----
        homing_type: string
            Either 'Guiding' or 'known'. Guiding pauses to let the operator manually adjust nozzle position, known uses pre-provided position and continues.

        Returns:
        -----

        '''
        if homing_type == 'Guiding':
            self.robot.robot_init_move()

            print("INFO: To enter guiding mode, EMERGENCY STOP button needs to be pressed DOWN and the robot light should be continuously WHITE!")
            print("After positioning robot, open the EMERGENCY STOP button to its UP position! The robot should the have BLUE lights")
            input("Position end-effector nozzle < 1cm from desired (0,0) location.\nWhen satisfied with position, press Enter to continue...")

            self.gcode_home_pose = self.robot.read_current_pose()
            self.gcode_home_pose_vec = self.gcode_home_pose.vector()

            # Can potentially add collision detection here to further improve home point. 
            # But new (0,0) point can be chosen from mid point of bed probing afterwards


        elif homing_type == 'known':
            print("Set gcode_home to this value. Home point assumed known")
            self.gcode_home_pose_vec = [0.48,0.0,0.0,-math.pi/3,0.0,0.0]

        else:
            print("Failed to home gcode zero... Check RobotMode input")

    def move_to_point(self,x,y,z):#todo
        '''
        Move to desired location, using x,y,z and G-code home pose offset and tool_pose reference frame

        Input:
        -----
        x: float
            X value of target position
        y: float
            Y value to target position
        z: float
            Z value of target position

        Returns:
        -----

        '''
        self.robot.set_dynamic_rel(0.1)
        z_compensation = self.vertical_bed_level_compensation((x + self.gcode_home_pose_vec[0],y + self.gcode_home_pose_vec[1],z + self.gcode_home_pose_vec[2]))
        motion = self.robot.make_linear_motion(self.robot.make_affine_object(x + self.gcode_home_pose_vec[0],y + self.gcode_home_pose_vec[1],z + self.gcode_home_pose_vec[2] + z_compensation))
        self.robot.execute_move(frame=self.robot.tool_frame,motion=motion)
        self.robot.recover_from_errors()

    def probe_bed(self):
        '''
        Probe the bed in a grid using motion force feedback to detect surface. Contact points are used to calculate the surface flatness.

        Input:
        -----

        Returns:
        -----
        contact_found: bool
            flag to inticate whether all probe points detected the surface or not. True if all points were successful.

        '''
        print("probing...")
        probe_locations_xy = [[[0.05,0.05],[0,0.05],[-0.05,0.05]],[[0.05,0],[0,0],[-0.05,0]],[[0.05,-0.05],[0,-0.05],[-0.05,-0.05]]] # relative to ghome_gcode

        bed_grid = [[[],[],[]],[[],[],[]],[[],[],[]]] # ugly ik

        contact_found = True

        for axis1 in range(3):
            for axis2 in range(3):

                self.robot.set_dynamic_rel(0.1)
                # Move to probe location
                affine1 = self.robot.make_affine_object(probe_locations_xy[axis1][axis2][0] + self.gcode_home_pose_vec[0],probe_locations_xy[axis1][axis2][1] + self.gcode_home_pose_vec[1],self.gcode_home_pose_vec[2]+0.05)
                m1 = self.robot.make_linear_motion(affine1)
                self.robot.execute_move(frame=self.robot.tool_frame,motion=m1)

                # Reset data reaction motion, may need to tweek trigger force when extruder is mounted
                #d2 = self.robot.make_Z_reaction_data(-2.0)
                d2 = MotionData().with_reaction(Reaction(Measure.ForceZ < -5.0))
                #d2 = MotionData().with_reaction(Reaction(Measure.ForceXYZNorm > 15.0))

                # Reduce dynamics for probing
                self.robot.set_dynamic_rel(0.02)

                # Move slowly towards print bed
                affine2 = self.robot.make_affine_object(0.1,0.0,-0.1)
                m2 = self.robot.make_linear_relative_motion(affine2)
                self.robot.execute_reaction_move(motion=m2,data=d2)
                #self.robot.robot.move(m2,d2)

                # Check if the reaction was triggered
                if d2.did_break:
                    self.robot.recover_from_errors()
                    print('Force exceeded 5N!')
                    print(f"Hit something for probe location x: {axis1}, y: {axis2}")

                    current_pose = self.robot.read_current_pose()
                    vector_pose = current_pose.vector()
                    probe_point = vector_pose[0:3]
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

        self.robot.recover_from_errors()

        self.robot.set_dynamic_rel(0.1)
        m1 = self.robot.make_linear_motion(self.robot.robot_home_pose)
        self.robot.execute_move(frame=self.robot.tool_frame,motion=m1)
        self.robot.recover_from_errors()

        return contact_found

    def calculate_bed_surface_plane(self):
        '''
        Calculate the plane equation given by ax + by + cz + d = 0 with points taken from the bed probing process. Uses 3 non-collinear points A, B and C and calculates the cross product AB x AC = [a,b,c] and d = -(aAx + bAy + cAz)

        Input:
        -----

        Returns:
        ----

        '''
        # Points
        A = self.bed_points[0][0]
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
        Calculates the vertical compensation needed due to non-horizontal build plane. Compensation is made based on z height equal to 0, making the z component of 'point' excessive and unused

        Input:
        -----
        point: [float,float,float]
            A point [x,y,z] for where compensation is calculated

        Returns:
        -----
        z_compensation: float
            A value corresponding to the offset from the bed plane in the Z direction at z=0

        '''
        return -(self.bed_plane_abcd[0]*point[0] + self.bed_plane_abcd[1]*point[1] + self.bed_plane_abcd[3])

    def does_model_fit_bed(self):
        '''
        Checks whether the physical dimension of  the model will fit in the build area

        Input:
        -----

        Returns:
        -----
        flag: bool
            True if model fits, else False

        '''
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
        '''
        Checks if build is feasible for the chosen G-code home point.

        Input:
        -----

        Returns:
        -----
        flag: bool
            True if feasible, else False

        '''
        if not self.does_model_fit_bed():
            print("Model does not fit on build plate")
            return False
        else: # skipping lower part for now
            return True

        # move in a square and detect collision
        edge_points = [[self.Xmax[1],self.Ymax[1],self.Zmax[0]+0.02],[self.Xmax[0],self.Ymax[1],self.Zmax[0]+0.02],[self.Xmax[0],self.Ymax[0],self.Zmax[0]+0.02],[self.Xmax[1],self.Ymax[0],self.Zmax[0]+0.02],[self.Xmax[1],self.Ymax[1],self.Zmax[0]+0.02]]
        reaction_motion = self.robot.make_linear_motion(self.robot.make_affine_object(self.robot.robot_home_pose_vec[0],self.robot.robot_home_pose_vec[1],self.robot.robot_home_pose_vec[2]))

        for point in edge_points:

            # Stop motion if the overall force is greater than 30N
            #data = MotionData().with_reaction(Reaction(Measure.ForceXYZNorm > 30.0, reaction_motion))
            data = self.robot.make_norm_reaction_data(30.0,reaction=reaction_motion)
            motion = self.robot.make_linear_motion(self.robot.make_affine_object(point[0] + self.robot.robot_home_pose_vec[0],point[1] + self.robot.robot_home_pose_vec[1],point[2]))

            self.robot.execute_move(frame=self.robot.tool_frame,motion=motion,data=data)

            if data.did_break:
                self.robot.recover_from_errors()
                print("Collision when checking build area.")
                return False

            reaction_motion = self.robot.make_linear_motion(self.robot.make_affine_object(point[0] + self.robot.robot_home_pose_vec[0],point[1] + self.robot.robot_home_pose_vec[1],point[2]))

        self.is_area_clear = True

        return True

    def turn_angle(self,line_number):
        '''
        Check if trajectory sharply changes direction
        '''
        prev_point = [self.prev_X,self.prev_Y,self.prev_Z]
        mid_point = [self.X,self.Y,self.Z]
        next_point = [self.X,self.Y,self.Z]

        if self.read_param(line_number,'X') != False:
            next_point[0] = (self.read_param(line_number,'X'))
        if self.read_param(line_number,'Y') != False:
            next_point[1] = (self.read_param(line_number,'Y'))
        if self.read_param(line_number,'Z') != False:
            next_point[2] = (self.read_param(line_number,'Z'))

        v1 = self.make_vector(np.array(prev_point),np.array(mid_point))
        v2 = self.make_vector(np.array(mid_point),np.array(next_point))

        if np.count_nonzero(v1) == 0 or np.count_nonzero(v2) == 0:
            return 0.0 # No direction change
        else:
            angle = self.angle_between(v1,v2)
            return angle

    def make_vector(self,start, end):
        return end - start

    def unit_vector(self,vector):
        """ Returns the unit vector of the vector.  """
        return vector / np.linalg.norm(vector)

    def angle_between(self,v1, v2):
        """ Returns the angle in radians between vectors 'v1' and 'v2'::

                >>> angle_between((1, 0, 0), (0, 1, 0))
                1.5707963267948966
                >>> angle_between((1, 0, 0), (1, 0, 0))
                0.0
                >>> angle_between((1, 0, 0), (-1, 0, 0))
                3.141592653589793
        """
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def make_waypoints(self,interval):
        '''
        Generate waypoint objects for motion trajectory to follow

        Input:
        -----
        interval: [int,int]
            Interval for which to make waypoints for 

        Returns:
        -----
        waypoints: list of Waypoint objects
            A list of waypoints in the Waypoint object type

        '''
        waypoints = []
        for point in range(interval[0],interval[1]+1):
            z_compensation = self.vertical_bed_level_compensation((self.read_param(point,'X') + self.gcode_home_pose_vec[0],self.read_param(point,'Y') + self.gcode_home_pose_vec[1],self.Z + self.gcode_home_pose_vec[2]))
            affine = self.robot.make_affine_object(self.read_param(point,'X') + self.gcode_home_pose_vec[0],self.read_param(point,'Y') + self.gcode_home_pose_vec[1],self.Z + self.gcode_home_pose_vec[2] + z_compensation)
            waypoint = self.robot.make_waypoint(affine)
            waypoints.append(waypoint)
        return waypoints

    def make_path(self,interval,corner_blending):
        '''
        Generate waypoints for path following with a blending distance for smoothing motion when changing taget waypoint

        Input:
        -----
        interval: [int,int]
            Interval for which to make the PathMotion object for
        corner_blend_threshold: float
            radius for whech a new waypoint should be used as motion target

        Returns:
        -----
        path: PathMotion object

        '''
        path_points = []
        for point in range(interval[0],interval[1]+1):
            for key in self.gcodelines[point].params:
                if key == 'X' or key == 'Y' or key == 'Z':
                    self.__dict__[key] = self.read_param(point,key)
            z_compensation = self.vertical_bed_level_compensation((self.X + self.gcode_home_pose_vec[0],self.Y + self.gcode_home_pose_vec[1],self.Z + self.gcode_home_pose_vec[2]))
            affine = self.robot.make_affine_object(self.X + self.gcode_home_pose_vec[0],self.Y + self.gcode_home_pose_vec[1],self.Z + self.gcode_home_pose_vec[2] + z_compensation)
            path_points.append(affine)
        path = self.robot.make_path_motion(path_points,corner_blending)
        return path

    def target_point(self,line_number):
        '''


        Input:
        -----
        line_number: int
            Integer index of target G-code line point

        Returns:
        -----
        The target point with bed level compensation and offset from G-code home location in relation to robot zero

        '''
        z_compensation = self.vertical_bed_level_compensation((self.read_param(line_number,'X') + self.gcode_home_pose_vec[0],self.read_param(line_number,'Y') + self.gcode_home_pose_vec[1],self.Z + self.gcode_home_pose_vec[2]))
        return self.read_param(line_number,'X') + self.gcode_home_pose_vec[0],self.read_param(line_number,'Y') + self.gcode_home_pose_vec[1],self.Z + self.gcode_home_pose_vec[2] + z_compensation

    def run_code_segments(self):
        for interval in self.list_of_intervals:
            line = interval[0]
            progress = math.floor(line/(self.number_of_lines - 1.0))
            print(f"Current progress is {progress}%")
            self.interval = interval
            self.run_code_segment()

    # Blocking action
    def run_code_segment(self):
        '''
        Processes one interval based on the Command in that interval. Machine settings, movement, extrusion is set based on command and parameters.

        Input:
        -----

        Returns:
        -----

        '''
        command = self.read_command(self.interval[0])
        # Handle different M (machine) commands
        if command[0] == 'M':
            # Call method for each M-command
            getattr(self,command,getattr(self,'default'))()

        elif command[0] == 'G':

            # Find current / new z-height %% Implement for each line instead of start of interval, ignored if no new value anyway %%
            if self.read_param(self.interval[0],'Z') != False:
                self.Z = self.read_param(self.interval[0],'Z')

            # Find desired feedrate / working speed / max velocity
            if self.read_param(self.interval[0],'F') != False:
                self.F = self.read_param(self.interval[0],'F')

            getattr(self,command,getattr(self,'default'))()

        else:
            print(f"Command other than M or G... Silently passing on command {command}")
            pass 

    def visualize_bed_mesh(self):
        '''
        Plot the bed mesh based on the bed_points from the probing sequence

        Input:
        -----

        Returns:
        -----

        '''
        try:
            bed_points = self.bed_points
        except:
            print("No bed points found, using default flat 3 x 3 surface")
            bed_points = [[[0.2,0.2,0.0],[0,0.2,0],[-0.2,0.2,0.0]],
                        [[0.2,0,0],[0,0,0.0],[-0.2,0,0.0]],
                        [[0.2,-0.2,0.0],[0,-0.2,0.0],[-0.2,-0.2,0.0]]]

        x_axis = len(bed_points)
        y_axis = len(bed_points[0])

        x = []
        y = []
        z = []

        max_z = bed_points[0][0][2]
        min_z = bed_points[0][0][2]

        for i in range(y_axis):
            temp_x = []
            temp_y = []
            temp_z = []

            for j in range(x_axis):
                temp_x.append(bed_points[i][j][0])
                temp_y.append(bed_points[i][j][1])
                temp_z.append(bed_points[i][j][2])

                if bed_points[i][j][2] > max_z:
                    max_z = bed_points[i][j][2]

                if bed_points[i][j][2] < min_z:
                    min_z = bed_points[i][j][2]

            x.append(temp_x)
            y.append(temp_y)
            z.append(temp_z)

        largest_axis = max([bed_points[0][0][0]-bed_points[-1][-1][0],bed_points[0][0][1]-bed_points[-1][-1][1],max_z-min_z])
        axis_scale = [(bed_points[0][0][0]-bed_points[-1][-1][0])/largest_axis,(bed_points[0][0][1]-bed_points[-1][-1][1])/largest_axis,(max_z-min_z)/largest_axis]
        
        fig = go.Figure(data=[go.Surface(z=z,x=x,y=y)])
        fig.update_traces(contours_z=dict(show=True, usecolormap=True,
                                        highlightcolor="limegreen", project_z=True))
        fig.update_layout(title='Surface flatness of print bed', autosize=False,
                        scene_camera_eye=dict(x=0.0, y=0.0, z=1.50),
                        width=1000, height=1000,
                        margin=dict(l=65, r=50, b=65, t=90),
                        scene=dict(
                            aspectratio = dict( x=axis_scale[0], y=axis_scale[1], z=axis_scale[2] ),
                            aspectmode = 'manual'
                        )
        )

        fig.show()

        #fig.write_image('bedmesh.eps',width=1920,height=1080)

    def visualize_gcode(self):
        '''
        Visualize motion trajectories where extrusion occures. Plot of extrusion paths generates a model

        Input:
        -----

        Returns:
        -----

        '''
        
        self.reset_parameters()

        # arrays for plotting data
        x_coordinates = []
        y_coordinates = []
        z_coordinates = []
        colors = []

        print("May have issues with 1 million+ points...")
        # Points here is vertices in the plot and corresponds to G-code lines

        for interval in self.list_of_intervals:
            if self.read_command(interval[0]) == 'G1':
                if (self.read_param(interval[0],'E') != False) and (self.read_param(interval[0],'E') > self.E) and (self.read_param(interval[0],'X') or self.read_param(interval[0],'Y') or self.read_param(interval[0],'Z')):
                    # front-pad the start position to the coming series of moves
                    x_coordinates.append(self.X * 1000)
                    y_coordinates.append(self.Y * 1000)
                    z_coordinates.append(self.Z * 1000)
                    colors.append(self.F)

                    if x_coordinates[-1] == x_coordinates[-3] and y_coordinates[-1] == y_coordinates[-3] and z_coordinates[-1] == z_coordinates[-3]:
                        x_coordinates = x_coordinates[:-2]
                        y_coordinates = y_coordinates[:-2]
                        z_coordinates = z_coordinates[:-2]
                        colors = colors[:-2]

                for point in range(interval[0],interval[1]+1):
                    if (self.read_param(point,'E') != False) and (self.read_param(point,'E') > self.E) and (self.read_param(interval[0],'X') or self.read_param(interval[0],'Y') or self.read_param(interval[0],'Z')):
                        for key in self.gcodelines[point].params:
                            self.__dict__[key] = self.read_param(point,key)

                        x_coordinates.append(self.X * 1000)
                        y_coordinates.append(self.Y * 1000)
                        z_coordinates.append(self.Z * 1000)
                        colors.append(self.F)

                    for key in self.gcodelines[point].params:
                        self.__dict__[key] = self.read_param(point,key)

                # To separate trajectories in the plot
                x_coordinates.append(None)
                y_coordinates.append(None)
                z_coordinates.append(None)
                colors.append(0)

            elif self.read_command(interval[0]) == 'G0':
                for point in range(interval[0],interval[1]+1):
                    for key in self.gcodelines[point].params:
                        self.__dict__[key] = self.read_param(point,key)

            else:
                for point in range(interval[0],interval[1]+1):
                    for key in self.gcodelines[point].params:
                        if self.read_param(point,'X') != False or self.read_param(point,'Y') != False or self.read_param(point,'Z') != False or self.read_param(point,'E') != False or self.read_param(point,'F') != False:
                            self.__dict__[key] = self.read_param(point,key)

        largest_axis = max([self.Xmax[1]-self.Xmax[0],self.Ymax[1]-self.Ymax[0],self.Zmax[1]-self.Zmax[0]])
        axis_scale = [(self.Xmax[1]-self.Xmax[0])/largest_axis,(self.Ymax[1]-self.Ymax[0])/largest_axis,(self.Zmax[1]-self.Zmax[0])/largest_axis]

        fig = go.Figure(data=go.Scatter3d(
            x=x_coordinates,y=y_coordinates,z=z_coordinates,
            mode="lines",
            line=dict(
                width=5,
                color=colors,
                cmin=0,
                cmax=self.Fmax[1],
                colorbar=dict(
                    borderwidth=0,
                    title=dict(
                        text='Feedrate [mm/min]'
                    )
                ),
                colorscale=[[0,'rgb(0,0,255)'],[1,'rgb(255,0,0)']]
                ),
            connectgaps=False
            ))

        fig.update_layout(
            autosize=True,
            scene=dict(
                aspectratio = dict( x=axis_scale[0], y=axis_scale[1], z=axis_scale[2] ),
                aspectmode = 'manual'
            ),
            title_text=('Visualization of extruded filament paths for ' + self.filename_),
            showlegend=False
        )

        fig.show()

        self.reset_parameters()

    def reset_parameters(self):
        '''
        Resets parameters to zero

        Input:
        -----

        Returns:
        -----

        '''
        # corresponding to 0,0,0 of start of printing
        self.X = 0
        self.Y = 0
        self.Z = 0
        self.F = 0
        self.E = 0

    def __str__(self):
        '''
        Sets the Class object string representation

        Input:
        -----

        Returns:
        -----
        "GCodeExecutor": string
            String representation of Class object

        '''
        return "GCodeExecutor"


    def display(self):
        '''
        Display basic iformation about the Class object such as filename of processed file, number of lines, material use, etc.

        Input:
        -----

        Returns:
        -----

        '''
        if '.gcode' in self.filename_:
            print(f"\nName of file being processed: {self.filename_}")
        else:
            print(f"\nName of file being processed: {self.filename_ + '.gcode'}")
        print(f"Number of command lines processed: {self.number_of_lines}")
        print("Total filament used: \n")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(add_help=True)

    parser.add_argument('--gfile',default='Circle')

    args = parser.parse_args()

    model = GCodeExecutor(args.gfile,{},{})
    model.load_gcode()
    model.visualize_gcode()
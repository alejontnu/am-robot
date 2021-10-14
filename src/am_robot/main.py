import os
import time
import sys
import argparse
import pandas as pd
import math

from gcodeparser import GcodeParser
from frankx import Affine, LinearRelativeMotion, Robot

from am_robot import utility
from am_robot import dynamics


# TODO - filename from argparse and automatic file locating
# Currently requires .gcode to be in data folder

def main():
    # Argument parsing
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description=('''
            Package for controlling a 3D printing on a 6 DoF robotic arm''')
            ,epilog='This is still under development',
            add_help=True)

    parser.add_argument('--host', default='10.0.0.2', help='FCI IP of the robot')
    parser.add_argument('--Gfile', default='dontprint', help='Gcode file name')
    args = parser.parse_args()

    # Initialize robot
    current_pose, robot, Connected = dynamics.init_robot(args.host)
    print(current_pose)

    current_location = [0,0,0]
    current_rotation = [math.pi/2,0,0]
    current_pose = [current_location[0],current_location[1],current_location[2],current_rotation[0],current_rotation[1],current_rotation[2]]
    target_location = [0,0,0]
    target_rotation = [math.pi/2,0,0]
    target_pose = [target_location[0],target_location[1],target_location[2],target_rotation[0],target_rotation[1],target_rotation[2]]

    # Input frequency
    Hz = 1000
    unit_divisor = 1000 # Default to mm
    home_point_offset = [0.480, 0.0, 0.40] # x,y,z coordinate of home point relative to base frame

    parseAll = True

    # Initialize extruder status
    am_geometry = utility.Geometry_Status(0,0,0,0,0,0,0,0,0,'idle')
    am_machine = utility.Machine_Status(0,0,0)

    # Gcode file location
    file_extrension = '.gcode'
    if file_extrension not in args.Gfile:
        filename = args.Gfile + '.gcode'
    folder = 'data'

    fullpath = os.path.join('.',folder,filename)

    # for root, dirs, files in os.walk(".", topdown=True):
    #     print('start')
    #     for dir in dirs:
    #         print(os.path.join(root, dir))
    #     print('stop-start')
    #     for file in files:
    #         print(os.path.join(root, file))
    #     print('stop')

    # Gcode load, read and parse
    with open(fullpath,'r') as file:
        gcodelines = GcodeParser(file.read()).lines
    
    '''
    # Single line version, slower as the parsing is done for each line inside the loop
    with open(fullpath,'r') as f:
        for index, linje in enumerate(f):
            #print("line {}: {}".format(index, linje.strip()))
            if linje[0] == 'G' or linje[0] == 'M':
                parsed_gline = GcodeParser(linje).lines
                pandalinje = pd.DataFrame(parsed_gline,columns=["command","params","comment"])
                Gline = utility.format_gcodeline(linje)
                print(parsed_gline)
                print(pandalinje)
                print(Gline)
    '''

    # Alternative Pandas dict format. Larger size (x40)          
    #GcodePandas = pd.DataFrame(gcodelines,columns=["command","params","comment"])

    # Example GcodeLine format
    # GcodeLine(
    #     command=('G',1),
    #     params={'X':12.3,'Y':7.4,'E':15.5},
    #     comment='Linear Move and Extrude')
    #
    # Gcode lengths are in mm or inches,and are therefor converted to meter by deviding by the unit_divisor

    for line in gcodelines:
        # Start time for checking loop time
        start_time = time.time()
        current_command = line.command
        current_params = line.params
        current_comment = line.comment

        if line.command[0] == 'G':
            am_geometry.gcommand = line.command[1]
            if line.command[1] == 0 or line.command[1] == 1:
                am_geometry.move_type = 'linear'
                for key in line.params:
                    #am_geometry.key = line.get_param(key)
                    if key == 'X': # abs or relative position
                        current_pose[0] = am_geometry.X
                        am_geometry.X = line.get_param(key)/unit_divisor
                        target_pose[0] = am_geometry.X
                    elif key == 'Y': # abs or relative position
                        current_pose[1] = am_geometry.Y
                        am_geometry.Y = line.get_param(key)/unit_divisor
                        target_pose[1] = am_geometry.Y
                    elif key == 'Z': # abs or relative position
                        current_pose[2] = am_geometry.Z
                        am_geometry.Z = line.get_param(key)/unit_divisor
                        target_pose[2] = am_geometry.Z
                    elif key == 'F': # Sets feedrate [mm/min]
                        am_geometry.F = line.get_param(key)/unit_divisor
                    elif key == 'E': # abs or relative position of filament [unit]
                        am_geometry.E = line.get_param(key)/unit_divisor

                # robot_movement(am_geometry)
                # update_extruder(am_geometry)

                dynamics.linear_move(current_pose,target_pose,am_geometry,robot)

            elif line.command[1] == 2 or line.command[1] == 3:
                if line.command[1] == 2:
                    am_geometry.move_type = 'cw_arc'
                else:
                    am_geometry.move_type = 'ccw_arc'
                for key in line.params:
                    if key == 'X':
                        current_pose[0] = am_geometry.X
                        am_geometry.X = line.get_param(key)/unit_divisor
                        target_pose[0] = am_geometry.X
                    elif key == 'Y':
                        current_pose[1] = am_geometry.Y
                        am_geometry.Y = line.get_param(key)/unit_divisor
                        target_pose[1] = am_geometry.Y
                    elif key == 'Z':
                        current_pose[2] = am_geometry.Z
                        am_geometry.Z = line.get_param(key)/unit_divisor
                        target_pose[2] = am_geometry.Z
                    elif key == 'F':
                        am_geometry.F = line.get_param(key)/unit_divisor
                    elif key == 'E':
                        am_geometry.E = line.get_param(key)/unit_divisor
                    elif key == 'R':
                        # TODO allow arc moves, or make them linear segments
                        print("R move - ignored")
                        #am_geometry.R = line.get_param(key)/unit_divisor
                    elif key == 'I':
                        print("I move - ignored")
                        #am_geometry.I = line.get_param(key)/unit_divisor
                    elif key == 'J':
                        print("J move - ignored")
                        #am_geometry.J = line.get_param(key)/unit_divisor
                    else:
                        am_geometry.__name__ = line.get_param(key)

                    #dynamics.curved_move(current_pose,target_pose,am_geometry)


            elif current_command[1] == 10: # Seems to not be used in favor of G1 commands doing the same
                print("start retraction move")
                am_geometry.move_type = 'retraction'
            elif current_command[1] == 11:
                print("Start recover move after a G10")
                am_geometry.move_type = recover
            elif current_command[1] == 20:
                unit_divisor = 39.37007874 # Inches per meter
                print("set inches")
            elif current_command[1] == 21:
                unit_divisor = 1000 # Millimeter per meter
                print("set millimeters")
            elif current_command[1] == 28:
                print("auto home move")
                am_geometry.move_type = 'home'
                if line.params == '':
                    am_geometry.X = 0
                    am_geometry.Y = 0
                    am_geometry.Z = 0
                else:
                    for key in line.params:
                        if key == 'X': # abs or relative position
                            am_geometry.X = 0
                        elif key == 'Y': # abs or relative position
                            am_geometry.Y = 0
                        elif key == 'Z': # abs or relative position
                            am_geometry.Z = 0
                        else:
                            print(f"No home move action for key: {key}")
            elif current_command[1] == 90:
                print("Set absolute positioning")
            elif current_command[1] == 91:
                print("Set relative positioning")
            elif current_command[1] == 92:
                print("Set positioning")
            else:
                print(f"No action for given G-command number: {current_command[1]}")
            
            #do stuff
        elif current_command[0] == 'M':
            am_machine.mcommand = current_command[1]
            if current_command[1] == 82:
                print("E absolute")
            elif current_command[1] == 84:
                print("Disable motors")
            elif current_command[1] == 104:
                print("Set hotend temperature")
            elif current_command[1] == 105:
                print("Report temperature")
            elif current_command[1] == 106:
                print("Set fan speed")
            elif current_command[1] == 107:
                print("Fan off")
            elif current_command[1] == 109:
                print("Wait for hotend temperature")
            elif current_command[1] == 140:
                print("Set bed temperature")
            else:
                print(f"No action for M command number: {current_command[1]}")
            #do other stuff
        else:
            print("Neither G nor M command")

        #print(str(am_geometry))

        end_time = time.time()
        if end_time-start_time > 0.0003: # Have about 300 us to spare to achieve 1kHz
            print(f"runtime loop = {end_time-start_time} > 300 us.")
        #time.sleep(0.5) # TODO - change to interval instead of pause
    

if __name__ == '__main__':
    main()
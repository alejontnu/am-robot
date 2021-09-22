import os
import time
from gcodeparser import GcodeParser

from am_robot import utility

# TODO - filename from argparse
# Currently requires .gcode to be in data folder

def main():

    # Initial extruder status
    am_status = utility.Status(0,0,0,0,0,0,1)
    am_machine = utility.Machine(0,0,0)
    print(str(am_status))
    print(str(am_machine))

    # Gcode file location
    filename = 'dontprint.gcode'
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

    # print(am_robot.__file__.split)

    # Gcode load and read
    with open(fullpath,'r') as file:
        gcode = file.read()

    # Parse gcode
    parsed_gcode = GcodeParser(gcode)

    # Examble GcodeLine format
    # GcodeLine(
    #     command=('G',1),
    #     params={'X':12.3,'Y':7.4,'E':15.5},
    #     comment='Linear Move and Extrude')

    for line in parsed_gcode.lines:
        
        start_time = time.time()

        current_command = line.command
        current_params = line.params
        current_comment = line.comment

        if current_command[0] == 'G':
            am_status.gcommand = current_command[1]
            if current_command[1] == 0 or current_command[1] == 1:
                am_status.move_type = 'linear'
                for key in current_params:
                    if key == 'X':
                        am_status.X = line.get_param(key)
                    elif key == 'Y':
                        am_status.Y = line.get_param(key)
                    elif key == 'Z':
                        am_status.Z = line.get_param(key)
                    elif key == 'F':
                        am_status.F = line.get_param(key)
                    elif key == 'E':
                        am_status.E = line.get_param(key)
            elif current_command[1] == 2 or current_command[1] == 3:
                if current_command[1] == 2:
                    am_status.move_type = 'cw_arc'
                else:
                    am_status.move_type = 'ccw_arc'
                for key in current_params:
                    if key == 'X':
                        am_status.X = line.get_param(key)
                    elif key == 'Y':
                        am_status.Y = line.get_param(key)
                    elif key == 'Z':
                        am_status.Z = line.get_param(key)
                    elif key == 'F':
                        am_status.F = line.get_param(key)
                    elif key == 'E':
                        am_status.E = line.get_param(key)
                    elif key == 'R':
                        # TODO allow arc moves, or make them linear segments
                        print("R move - ignored")
                        #am_status.R = line.get_param(key)
                    elif key == 'I':
                        print("I move - ignored")
                        #am_status.I = line.get_param(key)
                    elif key == 'J':
                        print("J move - ignored")
                        #am_status.J = line.get_param(key)
            elif current_command[1] == 10:
                print("start retraction move")
            elif current_command[1] == 11:
                print("Start recover move after a G10")
            elif current_command[1] == 20:
                print("set inches")
            elif current_command[1] == 21:
                print("set millimeters")
            elif current_command[1] == 28:
                print("auto home move")
            elif current_command[1] == 90:
                print("Set absolute positioning")
            elif current_command[1] == 91:
                print("Set relative positioning")
            elif current_command[1] == 92:
                print("Set positioning")
            else:
                print("No action for given G-command number")
            
            #do stuff
        elif current_command[0] == 'M':
            am_machine.mcommand = current_command[1]
            #do other stuff
        else:
            print("Neither G nor M command")

        print(str(am_status))

        end_time = time.time()
        if end_time-start_time > 0.0003: # Have about 300 us to spare to achieve 1kHz
            print(f"runtime loop = {end_time-start_time} > 300 us.")
        #time.sleep(0.5) # TODO - change to interval instead of pause

if __name__ == '__main__':
    main()
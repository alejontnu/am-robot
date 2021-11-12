import sys

if sys.platform == 'linux':
    from frankx import Robot
elif sys.platform == 'win32':
    try:
        from frankx import Robot    
    except Exception as e:
        print(e)
    finally:
        print('Running on OS: ' + sys.platform)


print(sys.platform)
import argparse

from am_robot import GCodeExecutor
from am_robot import ExtruderTool
from am_robot import dynamics

def main():
    '''
    Helper function that converts a parsed Gcode line into a dict structure

    Parameters:
    -----------

    Returns:
    -----------
    
    '''

    ''' Parsing input arguments '''
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description=('''
            Package for controlling a 3D printing on a 6 DoF robotic arm''')
            ,epilog='This is still under development',
            add_help=True)

    parser.add_argument('--host', default='10.0.0.2', help='FCI IP of the robot')
    parser.add_argument('--gfile', default='Circle', help='Gcode file name')
    parser.add_argument('--visualize', default=False, help='Visualize the given Gcode as a 3D plot. Skips any hardware connection precess')
    parser.add_argument('--skip_connection', default=False, help='If True, skips the connection to robot. For testing out-of-lab. Alse defaults too True if visualize is True')
    args = parser.parse_args()

    if args.visualize:
        args.skip_connection = True
    
    extruder_tool = ExtruderTool.ExtruderTool('FDM','10.0.0.3')
    robot, current_pose, is_connected = dynamics.init_robot(args.host,args.skip_connection)
    executor = GCodeExecutor.GCodeExecutor(args.gfile,robot,extruder_tool)
    executor.load_gcode()

    if args.visualize:
        executor.display()
        executor.visualize_gcode_plotly()

    #executor.run_code_segment()

if __name__ == '__main__':
    main()
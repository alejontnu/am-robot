# import sys
import argparse
import time

from am_robot.GCodeExecutor import GCodeExecutor
from am_robot.ExtruderTool import ExtruderTool
from am_robot.FrankaRobot import FrankaRobot

# if sys.platform == 'linux':
#     from frankx import Robot
# elif sys.platform == 'win32':
#     try:
#         from frankx import Robot
#     except Exception as e:
#         print(e)
#     finally:
#         print('Running on OS: ' + sys.platform)


def main():
    '''
    Additive manufactureing package for the Franka Emika Panda robot manipulator.

    Arguments:
    -----------
    host: string
        ip string used for robot connection (default: 10.0.0.2)
    tool: string
        serial string used for tool connection (default: /dev/ttyUSB0)
    gfile: string
        string name of gcode file used for additive manufacturing, with or without .gcode ending (default: Circle.gcode)
    visualize: bool, optional
        flag for enabling visualization for gcode (default: False)
    skip_connection: bool, optional
        flag for skipping connection with robot, useful when hardware is not connected (default: False)

    Returns:
    -----------
    3D object or visualization of object
    '''

    ''' Parsing input arguments '''
    parser = argparse.ArgumentParser(
        formatter_class=argparse.MetavarTypeHelpFormatter,
        description=('''Package for controlling a 3D printing on a 6 DoF robotic arm'''),
        epilog='This is still under development',
        add_help=True)

    parser.add_argument('--host', default='10.0.0.2', type=str, help='FCI IP of the robot')
    parser.add_argument('--tool', default='/dev/ttyUSB0', type=str, help='Serial connection of the tool used')
    parser.add_argument('--home_mode', default='Guiding', type=str, help='Mode type for homing to (0,0) of Gcode point. Guiding to manually position end-effector nozzle')
    parser.add_argument('--gfile', default='Circle.gcode', type=str, help='Gcode file name')

    parser.add_argument('--t_tool', default=[0,0,-0.1], type=list, help='Translation due to Tool as [x,y,z]')
    parser.add_argument('--d_nozzle', default=0.8, type=float, help='Hot-End Nozzle diameter')
    parser.add_argument('--f_width', default=2.85, type=float, help='Width of filament used')

    parser.add_argument('--visualize', action='store_true', help='If specified, visualize the given Gcode as a 3D plot')
    parser.add_argument('--lines', default=100000000, type=int, help='Max number of lines to process, default is higher than ever expected')
    parser.add_argument('--skip_connection', action='store_true', help='If specified, skips the connection to robot. For testing out-of-lab. Also defaults too True if visualize is True')
    parser.add_argument('--skip_probe', action='store_true', help='If specified, skips the bed probing step')
    parser.add_argument('--skip_segments', action='store_true', help='If specified, skips the G-code segments')
    args = parser.parse_args()

    time_elapsed_task = time.time()
    time_elapsed_total = time.time()

    tool = ExtruderTool(args.tool,'FDM',args.f_width,args.d_nozzle,args.t_tool,args.skip_connection)
    robot = FrankaRobot(args.host,args.skip_connection)
    executor = GCodeExecutor(args.gfile,robot,tool)
    executor.load_gcode(args.lines)

    print("Done pre-processing gcode")

    if args.visualize:
        time_elapsed_task = time.time()
        executor.display()
        executor.visualize_gcode()
        time_elapsed_task = time.time() - time_elapsed_task

        print(f"Visualization done in {time_elapsed_task:.5f}s")

        input("Press Enter to continue if satisfied with model plot...")

    if executor.robot.is_connected:
        # Manually position end effector/extrusion nozzle at 'home' point
        executor.home_gcode(args.home_mode)

        # Check bounds for build area
        # proceed = executor.is_build_feasible()
        # executor.robot.gripper.clamp(0.005)

        # Uses force feedback to determine where n points of the print bed are located
        if not args.skip_probe:
            bed_found = executor.probe_bed()
        else:
            bed_found = True
            executor.bed_plane_transformation_matrix = executor.rotation_matrix()

        if bed_found and not args.skip_segments:
            # Make a bed mesh for knowing the surface flatness and location of build area
            if args.visualize:
                executor.visualize_bed_mesh()
                input("When happy with bed mesh press enter...")

            time_elapsed_task = time.time()

            try:
                executor.run_code_segments()
            except KeyboardInterrupt:
                executor.tool.set_feedrate(0.0)
                executor.tool.set_nozzletemp(0.0)
                exit()

            time_elapsed_task = time.time() - time_elapsed_task

        else:
            print("One of more points of the bed was not found, check and level bed roughly")

    time_elapsed_total = time.time() - time_elapsed_total

    print(f"Task done in {time_elapsed_task:.5f}s")
    print(f"Total time elapsed: {time_elapsed_total:.5f}s")

    if not args.skip_connection:
        executor.tool.set_feedrate(0.0)
        executor.tool.disconnect()


if __name__ == '__main__':
    main()

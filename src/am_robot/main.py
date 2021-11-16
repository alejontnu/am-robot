import sys
import argparse
import time

if sys.platform == 'linux':
    from frankx import Robot
elif sys.platform == 'win32':
    try:
        from frankx import Robot    
    except Exception as e:
        print(e)
    finally:
        print('Running on OS: ' + sys.platform)


from am_robot import GCodeExecutor
from am_robot import ExtruderTool
from am_robot import FrankaRobot

def main():
    '''
    Helper function that converts a parsed Gcode line into a dict structure

    Parameters:
    -----------

    Returns:
    -----------
    
    '''

    ''' Parsing input arguments '''
    parser = argparse.ArgumentParser(formatter_class=argparse.MetavarTypeHelpFormatter,
        description=('''Package for controlling a 3D printing on a 6 DoF robotic arm''')
        ,epilog='This is still under development',
        add_help=True)

    parser.add_argument('--host', default='10.0.0.2', type=str, help='FCI IP of the robot')
    parser.add_argument('--tool', default='10.0.0.3', type=str, help='Connection of the tool used')
    parser.add_argument('--home_mode', default='Guiding', help='Mode type for homing to (0,0) of Gcode point. Guiding to manually position end-effector nozzle')
    parser.add_argument('--gfile', default='Circle.gcode', type=str, help='Gcode file name')
    parser.add_argument('--t_tool', default=[0,0,-0.1], type=list, help='Translation due to Tool as [x,y,z]')
    parser.add_argument('--d_nozzle', default=0.8, type=float, help='Hot-End Nozzle diameter')
    parser.add_argument('--f_width', default=2.85, type=float, help='Width of filament used')
    parser.add_argument('--visualize', default=False, type=bool, help='Visualize the given Gcode as a 3D plot. Skips any hardware connection precess')
    parser.add_argument('--skip_connection', default=False, type=bool, help='If True, skips the connection to robot. For testing out-of-lab. Alse defaults too True if visualize is True')
    args = parser.parse_args()

    time_elapsed_task = time.time()
    time_elapsed_total = time.time()

    #if args.visualize:
    #    args.skip_connection = True
    
    extruder_tool = ExtruderTool.ExtruderTool('FDM','10.0.0.3',args.f_width,args.d_nozzle,args.t_tool)
    robot = FrankaRobot.FrankaRobot(args.host,args.skip_connection)
    executor = GCodeExecutor.GCodeExecutor(args.gfile,robot,extruder_tool)
    executor.load_gcode()

    print(executor.list_of_intervals)

    print("Done pre-processing gcode")

    if args.visualize:
        time_elapsed_task = time.time()
        executor.display()
        executor.visualize_gcode()
        time_elapsed_task = time.time() - time_elapsed_task
    

    if executor.robot.is_connected:
        # Manually position end effector/extrusion nozzle at 'home' point
        executor.home_gcode(args.home_mode)
        
        # Check bounds for build area
        #proceed = executor.is_build_feasible()
        
        # Uses force feedback to determine where n points of the print bed are located
        #if proceed:
        #executor.probe_bed()
        
        # Make a bed mesh for knowing the surface flatness and location of build area
        #executor.construct_bed_mesh()
        
        count = 0
        
        time_elapsed_task = time.time()
        for interval in executor.list_of_intervals:
            # Blocking function:
            print(executor.list_of_intervals[count])
            executor.run_code_segment(interval)
            count = count + 1
            if count > 14:
                break
        time_elapsed_task = time.time() - time_elapsed_task
        
    time_elapsed_total = time.time() - time_elapsed_total

    print(f"Task done in {time_elapsed_task:.5f}s")
    print(f"Total time elapsed: {time_elapsed_total:.5f}s")

if __name__ == '__main__':
    main()
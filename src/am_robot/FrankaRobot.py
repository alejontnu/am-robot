import sys
import math

if sys.platform == 'linux':
    from frankx import Affine, LinearMotion, Robot, RobotMode, RobotState, WaypointMotion, JointMotion
elif sys.platform == 'win32':
    try:
        from frankx import Affine, LinearMotion, Robot, RobotMode, RobotState, WaypointMotion, JointMotion
    except Exception as e:
        print(e)
    finally:
        print('Running on OS: ' + sys.platform)

'''
change to inheritance 
class Robot(ConnectedRobot)
    def __init__(self,...)
        super.__init__(...)
where Robot is the abstract and ConnectedRobot is robot class used i.e. Robot from Frankx
'''


class FrankaRobot:
    def __init__(self,_host,_skip_connection):
        # To skip connection (True) when not at robot for testing other functions without connection timeout

        self.ip = _host

        if _skip_connection == True:
            print("Skipped trying to connect to robot with IP: " + self.ip)
            self.robot = {}
            self.current_pose = []
            self.is_connected = False

        else:
            try:
                print("Attempting to connect to robot...")
                self.robot = Robot(self.ip,repeat_on_error=False)
            except Exception as e:
                print("Could not connect to robot on IP: " + self.ip + "\n")
                raise e 
            else:
                print("Connected to robot in IP: " + self.ip)
                print(self.robot)

                self.is_connected = True

        self.radius = 0.855
        self.height_up = 1.190
        self.height_down = -0.360
        self.sweep_angle = 2 * math.pi
        self.base_area = 0.1 # front area radius

    def set_robot_mode(self,mode):
        if mode == 'Guiding':
            RobotMode.Guiding
        elif mode == 'idle':
            RobotMode.Idle

    def read_Robot_state(self,state):
        return RobotState.__dict__[state]

    def lin_move_to_point(self,X,Y,Z):
        move = LinearMotion(Affine(X,Y,Z))
        self.robot.move(move)

    def follow_waypoints(self,waypoints):
        motion = WaypointMotion(waypoints,return_when_finished=False)
        thread = self.robot.move_async(motion)
        self.robot.move()

    def robot_home_move(self):
        self.robot.set_default_behavior()

        # Recover from errors
        self.robot.recover_from_errors()

        # Set acceleration and velocity reduction
        self.robot_dynamic_rel = 0.2
        self.robot.set_dynamic_rel(self.robot_dynamic_rel) # Default 0.1

        # Joint motion to set initial configuration
        self.robot.move(JointMotion([0.0, 0.4, 0.0, -2.0, 0.0, 2.4, 0.0]))

        # Define and move to cartesian space
        self.tool_frame = Affine(z=-0)
        self.robot_home_pose = Affine(0.500, 0.0, 0.40) # NB not same as auto-home extruder
        self.robot_home_pose_vec = [0.500,0.0,0.40]

        #self.robot.move(self.tool_frame, LinearMotion(self.robot_home_pose))

        self.home_pose = self.read_current_pose()

    def read_current_pose(self):
        return self.robot.current_pose()
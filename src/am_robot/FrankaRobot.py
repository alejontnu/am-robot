import sys
import math
import numpy as np

from am_robot.AbstractRobot import AbstractRobot

if sys.platform == 'linux':
    from frankx import Affine, LinearMotion, Robot, RobotMode, RobotState, WaypointMotion, JointMotion, Waypoint, Reaction, LinearRelativeMotion, Measure, PathMotion, MotionData
    from _movex import Path, TimeParametrization, Trajectory
elif sys.platform == 'win32':
    try:
        from frankx import Affine, LinearMotion, Robot, RobotMode, RobotState, WaypointMotion, JointMotion, Waypoint, Reaction, LinearRelativeMotion, Measure, PathMotion, MotionData
    except Exception as e:
        print(e)
    finally:
        print('Running on OS: ' + sys.platform)


class FrankaRobot(AbstractRobot):
    '''
    Class object attributes and methods for a Franka Emika Panda robot.

    Attributes:
    -----------
    host: string
        ip string for cennection to robot. (Default: 10.0.0.2)
    skip_connection: bool
        flag to enable skipping connection if desired. (Default: False)

    Methods:
    -----------
    ...

    '''
    def __init__(self, host, skip_connection):
        '''/ 40.0
        Initialize the Class object

        Input:
        -----
        host: string
            ip string for cennection to robot. (Default: 10.0.0.2)
        skip_connection: bool
            flag to enable skipping connection if desired. (Default: False)

        Returns:
        -----
        Initialized Class object

        '''
        super().__init__(host)

        self.host = host


        # To skip connection (True) when not at robot for testing other functions without connection timeout
        if skip_connection is True:
            print("Skipped trying to connect to robot with IP: " + self.host)
            self.robot = {}
            self.current_pose = []
            self.is_connected = False

        else:
            try:
                print("Attempting to connect to robot...")
                self.robot = Robot(self.host, repeat_on_error=False)
                self.max_cart_vel = self.robot.max_translation_velocity  # m/s max cartesian velocity
                self.max_cart_acc = self.robot.max_translation_acceleration  # m/s² max cartesian acceleration
                self.max_cart_jerk = self.robot.max_translation_jerk  # m/s³ max cartesion jerk
            except Exception as e:
                print("Could not connect to robot on host IP: " + self.host + "\n")
                raise e
            else:
                print("Connected to robot on host IP: " + self.host)

                self.is_connected = True
                # self.gripper = self.robot.get_gripper()

        # Working area of robot
        self.radius = 0.855
        self.height_up = 1.190
        self.height_down = -0.360
        self.sweep_angle = 2 * math.pi
        self.base_area = 0.1  # front area radius

    def set_robot_mode(self, mode):
        '''
        Change mode of robot, i.e. Guiding, idle, etc

        Input:
        -----
        mode: string
            string for the type of mode the robot should be in

        Returns:
        -----

        '''
        if mode == 'Guiding':
            RobotMode.Guiding
        elif mode == 'idle':
            RobotMode.Idle

    def read_Robot_state(self, state):
        '''
        Return the desired state information from RobotState object

        Input:
        -----10.0,10.0,10.0,10.0,10.0,10.
        Returns:
        -----
        RobotState.state: varies
            Value of desired robot state

        '''
        return RobotState.__dict__[state]

    def lin_move_to_point(self, X, Y, Z):
        '''
        Move linearly to a target X,Y,Z point

        Input:
        -----

        Returns:
        -----

        '''
        move = LinearMotion(Affine(X, Y, Z))
        self.robot.move(self.tool_frame, move)

    def follow_waypoints(self, waypoints):
        '''
        Perform a waypoint move to follow a list of waypoints in a sepatate thread

        Input:
        -----
        waypoints: list of Waypoints
            A list of Waypoint object target points

        Returns:
        -----
        motion: WaypointMotion object
            The movement object used by Frankx
        thread: Thread
            The thread which the movement is performed in. To be joined later

        '''
        motion = WaypointMotion(waypoints, return_when_finished=False)
        thread = self.robot.move_async(motion)
        self.robot.move()
        return motion, thread

    def robot_init_move(self):
        '''
        The initial move the robot performs before manual positioning of G-code home point. To orient joints such that they are positioned in a desired orientation.
        Initial postition values are also set, such as tool_frame vector and pose used for reference frame of movements for the end-effector.

        Input:
        -----

        Returns:
        -----

        '''
        self.set_default_behavior()

        # Recover from errors
        self.recover_from_errors()

        # Set acceleration and velocity reduction
        self.robot_dynamic_rel = 0.1
        self.set_dynamic_rel(self.robot_dynamic_rel)  # Default 0.1

        # Defining tool_frame
        # self.tool_frame = Affine(0.03414, -0.0111, -0.09119, 0.0, -math.pi/4, 0.0)
        # self.tool_frame_vector = [0.03414, -0.0111, -0.09119, 0.0, -math.pi/4, 0.0]
        self.tool_frame = Affine(0.03414, -0.0111, -0.09119, 0.0, -math.pi/4, 0.0)
        self.tool_frame_vector = [0.03414, -0.0111, -0.09119, 0.0, -math.pi/4, 0.0]

        # Joint motion to set initial configuration
        self.robot.move(JointMotion([0.0, 0.4, 0.0, -2.0, 0.0, 2.4, 0.0]))

        # Set this within 5 cm of build plate for testing so that probing can start close enough without having to manually mode end-effectior into position. NB! Find out desired location
        self.robot_home_pose = Affine(0.350, 0.0, -0.06)  # NB not same as auto-home extruder
        self.robot_home_pose_vec = [0.350, 0.0, -0.06]

        # Position tool head
        self.robot.move(self.tool_frame, LinearMotion(self.robot_home_pose))

        self.home_pose = self.read_current_pose()
        print(self.home_pose.vector())
        self.set_velocity_rel(0.05)
        self.set_acceleration_rel(0.05)
        self.set_jerk_rel(0.02)

    def read_current_pose(self):
        '''
        Return the current pose of the robot taking the self.tool_frame into account

        Input:
        -----

        Returns:
        -----
        pose: Affine object
            The pose of the robot as Frankx' Affine object type

        '''
        return self.robot.current_pose(self.tool_frame)

    def set_default_behavior(self):
        self.robot.set_default_behavior()

    def recover_from_errors(self):
        self.robot.recover_from_errors()

    def set_dynamic_rel(self, value):
        self.robot.set_dynamic_rel(value)

    def set_velocity_rel(self, value):
        self.robot.velocity_rel = value

    def set_acceleration_rel(self, value):
        self.robot.acceleration_rel = value

    def set_jerk_rel(self, value):
        self.robot.jerk_rel = value

    def execute_move(self, frame=None, motion=None):
        if frame is None:
            self.robot.move(motion)
        else:
            self.robot.move(frame, motion)

    def execute_reaction_move(self, frame=None, motion=None, data=None):
        if frame is None:
            self.robot.move(motion, data)
        else:
            self.robot.move(frame, motion, data)

    def execute_threaded_move(self, frame=Affine(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), motion=None, data=None):
        if data is None:
            thread = self.robot.move_async(frame, motion)
            return thread
        else:
            thread = self.robot.move_async(frame, motion, data)
            return thread

    def make_linear_motion(self, affine):
        return LinearMotion(affine)

    def make_linear_relative_motion(self, affine):
        return LinearRelativeMotion(affine)

    def make_Z_reaction_data(self, force, reaction=None):
        return MotionData().with_reaction(Reaction(Measure.ForceZ < force, reaction))

    def make_norm_reaction_data(self, force, reaction=None):
        return MotionData().with_reaction(Reaction(Measure.ForceXYZNorm > force, reaction))

    def make_affine_object(self, x, y, z, a=0, b=0, c=0):
        return Affine(x, y, z, a, b, c)

    def set_dynamic_motion_data(self, dynamic_rel):
        return MotionData(dynamic_rel)

    def make_waypoint(self, affine,velocity_rel=1.0):
        waypoint = Waypoint(affine)
        waypoint["velocity_rel"] = velocity_rel
        return waypoint

    def make_path_motion(self, path_points, blending_distance):
        path_motion = PathMotion(path_points, blend_max_distance=blending_distance)
        path = Path(path_points, blend_max_distance=blending_distance)
        return path_motion, path

    def parametrize_path(self,path,timestep,vel_rels,accel_rels,jerk_rels):
        tp = TimeParametrization(delta_time=timestep)
        trajectory = tp.parametrize(path, vel_rels, accel_rels, jerk_rels)
        t_list, s_list, v_list, a_list, j_list = [], [], [], [], []
        for state in trajectory.states:
            t_list.append(state.t)
            s_list.append(state.s)
            v_list.append(state.ds)
            a_list.append(state.dds)
            j_list.append(state.ddds)
        return np.array(t_list), np.array(s_list), np.array(v_list), np.array(a_list), np.array(j_list)

from frankx import Affine, LinearMotion, Robot

class FrankaRobot:
    def __init__(self,_host,_skip_connection):
        # To skip connection (True) when not at robot for testing other functions without connection timeout

        self.ip = _host

        if _skip_connection:
            print("Skipped trying to connect to robot with IP: " + self.ip)
            self.robot = {}
            self.current_pose = []
            self.is_connected = False

        else:
            try:
                print("Attempting to connect to robot...")
                self.robot = Robot(robot_ip,repeat_on_error=False)
            except Exception as e:
                print("Could not connect to robot on IP: " + self.ip)
                raise e 
            else:
                print("Connected to robot in IP: " + self.ip)
                print(robot)

                self.is_connected = True


    def robot_home_move(self):
        self.robot.set_default_behavior()

        # Recover from errors
        self.robot.recover_from_errors()

        # Set acceleration and velocity reduction
        self.robot_dynamic_rel = 0.05
        self.robot.set_dynamic_rel(self.robot_dynamic_rel) # Default 0.1

        # Joint motion - Wierd error here...
        #robot.move(JointMotion([-1.811944, 1.179108, 1.757100, -2.14162, -1.143369, 1.633046, -0.432171]))

        # Define and move forwards
        self.camera_frame = Affine(y=0.05)
        self.robot_home_pose = Affine(0.480, 0.0, 0.40) # NB not same as auto-home extruder

        self.robot.move(self.camera_frame, LinearMotion(self.robot_home_pose, 1.75))

    def get_current_pose(self):
        self.current_pose = robot.current_pose()
        return self.current_pose
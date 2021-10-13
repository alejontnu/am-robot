from frankx import Affine, LinearRelativeMotion, Robot

def init_robot(robot_ip):

	# To skip connection when not at robot for testing other functions without connection timeout
	try_to_connect = False
	if not try_to_connect:
		print("Skipped trying to connect to robot with IP: " + robot_ip)
		return 0,0

	try:
		print("Attempting to connect to robot...")
		robot = Robot(robot_ip)
	except Exception as e:
		print("Could not connect to robot on IP: " + robot_ip)
		raise e	
	else:
		print("Connected to robot in IP: " + robot_ip)
		robot.set_default_bahavior()

		# Recover from errors
		robot.recover_from_errors()

		# Set acceleration and velocity reduction
		robot.set_dynamic_rel(0.05) # Default 0.1

		# Joint motion - Wierd error here...
		#m_joint = JointMotion([-1.811944, 1.179108, 1.757100, -2.14162, -1.143369, 1.633046, -0.432171])
	    #robot.move(m_joint)

		# Define and move forwards
		camera_frame = Affine(y=0.05)
		home_pose = Affine(0.480, 0.0, 0.40) # NB not same as auto-home extruder

		robot.move(camera_frame, LinearMotion(home_pose, 1.75))

		# Get the current pose
		current_pose = robot.current_pose()

		return current_pose, robot

'''
motion = LinearRelativeMotion(Affine(0.2, 0.0, 0.0))
robot.move(motion)

z_translation = Affine(0.0, 0.0, 0.5)
z_rotation = Affine(0.0, 0.0, 0.0, math.pi / 3, 0.0, 0.0)
combined_transformation = z_translation * z_rotation

# These two are now numpy arrays
euler_angles = combined_transformation.angles()
pose = combined_transformation.vector()

# A point-to-point motion in the joint space
m1 = JointMotion([-1.81194, 1.17910, 1.75710, -2.1416, -1.14336, 1.63304, -0.43217])

# A linear motion in cartesian space
m2 = LinearMotion(Affine(0.2, -0.4, 0.3, math.pi / 2, 0.0, 0.0))
m3 = LinearMotion(Affine(0.2, -0.4, 0.3, math.pi / 2, 0.0, 0.0), elbow=1.7)  # With target elbow angle

# A linear motion in cartesian space relative to the initial position
m4 = LinearRelativeMotion(Affine(0.0, 0.1, 0.0))

# A more complex motion by defining multiple waypoints
m5 = WaypointMotion([
  Waypoint(Affine(0.2, -0.4, 0.2, 0.3, 0.2, 0.1)),
  # The following waypoint is relative to the prior one
  Waypoint(Affine(0.0, 0.1, 0.0), Waypoint.ReferenceType.Relative)
])

# Hold the position for [s]
m6 = PositionHold(5.0)

robot.move(m1)
robot.move(m2)

# To use a given frame relative to the end effector
camera_frame = Affine(0.1, 0.0, 0.1)
robot.move(camera_frame, m3)

# To change the dynamics of the motion, use MotionData
data = MotionData(0.2)  # Using a dynamic_rel of 0.2 (eventually multiplied with robot.dynamic_rel)
robot.move(m4, data)

data.velocity_rel = 1.0
data.jerk_rel = 0.2
'''
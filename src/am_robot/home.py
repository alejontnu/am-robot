from frankx import Robot, Affine, LinearMotion, JointMotion, PathMotion, RobotState

robot = Robot('10.0.0.2',repeat_on_error=False)

robot.set_default_behavior()

# Recover from errors
robot.recover_from_errors()

# Set acceleration and velocity reduction
robot_dynamic_rel = 0.2
robot.set_dynamic_rel(robot_dynamic_rel) # Default 0.1

# Joint motion - Wierd error here...
#robot.move(JointMotion([-1.811944, 1.179108, 1.757100, -2.14162, -1.143369, 1.633046, -0.432171]))
robot.move(JointMotion([0.0, 0.4, 0.0, -2.0, 0.0, 2.4, 0.0])) # forward arc shape in X axis

# Define and move forwards
tool_frame = Affine(z=-0.05)
robot_home_pose = Affine(0.500, 0.0, 0.40) # NB not same as auto-home extruder
robot_home_pose_vec = [0.500,0.0,0.40]

robot.set_dynamic_rel(0.05)

print(type(RobotState).O_T_EE.__get__(RobotState,type(RobotState)))

path = PathMotion([
    Affine(0.5,-0.2,0.4),
    Affine(0.5,0.0,0.4),
    Affine(0.5,0.2,0.4),
    Affine(0.5,0.2,0.3),
    Affine(0.5,0.0,0.3)],
    blend_max_distance=0.001)
robot.move(path)



home_pose = robot.current_pose()

robot.move(tool_frame, LinearMotion(home_pose))
from frankx import Robot, Affine, LinearMotion, JointMotion, PathMotion, RobotState

import math

robot = Robot('10.0.0.2',repeat_on_error=False)
print(robot)
robot.set_default_behavior()
print(robot)
# Recover from errors
robot.recover_from_errors()
print(robot)
# Set acceleration and velocity reduction
robot_dynamic_rel = 0.2
robot.set_dynamic_rel(robot_dynamic_rel)  # Default 0.1
print(robot_dynamic_rel)

print(robot.current_pose())

affine = Affine(0.3,0,0.15)

print(affine)

motion = LinearMotion(affine)

print(motion)

robot.move(motion)


# # Joint motion - Wierd error here..
# # robot.move(JointMotion([-1.811944, 1.179108, 1.757100, -2.14162, -1.143369, 1.633046, -0.432171]))
# robot.move(JointMotion([0.0, 0.4, 0.0, -2.0, 0.0, 2.4, 0.0]))  # forward arc shape in X axis

# # Define and move forwards
# tool_frame = Affine(-0.03414,-0.0111,-0.1033,0.0,-math.pi/4,0.0)

# tool_frame_vector = [-0.03414,-0.0111,-0.1033, 0.0,-math.pi/4,0.0]
# robot_home_pose = Affine(0.300, 0.0, 0.0)  # NB not same as auto-home extruder
# robot_home_pose_vec = [0.500,0.0,0.40]

# robot.set_dynamic_rel(0.05)

# # print(type(RobotState).O_T_EE.__get__(RobotState,type(RobotState)))

# path = PathMotion([
#     Affine(0.3,-0.1,0.0),
#     Affine(0.3,0.0,0.0),
#     Affine(0.25,0.1,0.0),
#     Affine(0.3,0.1,0.0),
#     Affine(0.35,0.0,0.0)],
#     blend_max_distance=0.01)


# home_pose = robot.current_pose()

# robot.move(tool_frame, LinearMotion(robot_home_pose))

# robot_home_pose = Affine(0.300, 0.0, 0.0)

# robot.move(tool_frame, path)

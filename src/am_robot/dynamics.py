from frankx import Affine, LinearRelativeMotion, Robot
import frankx as fx
robot = Robot("10.0.0.2")
robot.set_dynamic_rel(0.05)

motion = LinearRelativeMotion(Affine(0.2, 0.0, 0.0))
robot.move(motion)

z_translation = Affine(0.0, 0.0, 0.5)
z_rotation = Affine(0.0, 0.0, 0.0, math.pi / 3, 0.0, 0.0)
combined_transformation = z_translation * z_rotation

# These two are now numpy arrays
euler_angles = combined_transformation.angles()
pose = combined_transformation.vector()
from frankx import Robot

from am_robot import GCodeExecutor
from am_robot import ExtruderTool

def main():
	# Argparsing

	extruder_tool = ExtruderTool.ExtruderTool('FDM','10.0.0.3')
	#robot = Robot('10.0.0.2')
	robot = {}
	executor = GCodeExecutor.GCodeExecutor('Circle',robot,extruder_tool)
	executor.run_code_segment()

if __name__ == '__main__':
	main()
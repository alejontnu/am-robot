import am_robot

class GCodeExecutor:
	interval = [0,0]
	def __init__(self,filename,interval):
		# Read and parse gcode into a full object variable
		file_extrension = '.gcode'
		if file_extrension not in args.Gfile:
			filename = args.Gfile + '.gcode'
		folder = 'data'

		fullpath = os.path.join('.',folder,filename)
		with open(fullpath,'r') as file:
			self.gcodelines = GcodeParser(file.read()).lines
		# scan through sections at a time
		self.interval = interval

	def run(self):
		return 0

	def code_segment(self):

		return 0

	def find_command_change(self):

		return interval

	def __str__(self):
		return "gcodeexecutor"
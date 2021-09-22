

class Status:
	def __init__(self,_gcommand,_x_pos,_y_pos,_z_pos,_ext_speed,_ext_volume,_move_type):
		self.gcommand = _gcommand
		self.X = _x_pos
		self.Y = _y_pos
		self.Z = _z_pos
		self.F = _ext_speed
		self.E = _ext_volume
		self.move_type = _move_type
		

	def __str__(self):
		return 'G command: G' + str(self.gcommand) + ', x pos: ' + str(self.X) + ", y pos: " + str(self.Y) + ", z pos: " + str(self.Z) + ", ext speed: " + str(self.F) + ", ext volume: " + str(self.E) + ", Movement type: " + str(self.move_type)

class Machine:
	def __init__(self,_mcommand,_nozzle_temp,_bed_temp):
		self.mcommand = _mcommand
		self.nozzle_temp = _nozzle_temp
		self.bed_temp = _bed_temp

	def __str__(self):
		return 'M command: M' + str(self.mcommand) + ", Nozzle temp: " + str(self.nozzle_temp) + ", Bed temp: " + str(self.bed_temp)

def gcode_action(x):
	print("gcode_action, "+x)
	return{
	'X': 1,
	'Y': 2,
	'Z': 3,
	'F': 4,
	'E': 555,
	}[x]
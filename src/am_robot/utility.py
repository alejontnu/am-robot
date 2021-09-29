

class Geometry_Status: # G-commands, related to geometry, movement type, extrusion volume, etc
	def __init__(self,_gcommand,_x_pos,_y_pos,_z_pos,_ext_speed,_ext_volume,_radius,_i_length,_j_length,_move_type):
		self.gcommand = _gcommand
		self.X = _x_pos
		self.Y = _y_pos
		self.Z = _z_pos
		self.F = _ext_speed
		self.E = _ext_volume
		self.R = _radius
		self.I = _i_length
		self.J = _j_length
		self.move_type = _move_type

	def __str__(self):
		return 'G command: G' + str(self.gcommand) + ', x pos: ' + str(self.X) + ", y pos: " + str(self.Y) + ", z pos: " + str(self.Z) + ", ext speed: " + str(self.F) + ", ext volume: " + str(self.E) + ", Arc radius: " + str(self.R) + ", Arc i-length: " + str(self.I) + ", Arc j-length: " + str(self.J) + ", Movement type: " + str(self.move_type)

class Machine_Status: # M-commands, related to machine settings, temperature, motors, etc
	def __init__(self,_mcommand,_nozzle_temp,_bed_temp):
		self.mcommand = _mcommand
		self.nozzle_temp = _nozzle_temp
		self.bed_temp = _bed_temp

	def __str__(self):
		return 'M command: M' + str(self.mcommand) + ", Nozzle temp: " + str(self.nozzle_temp) + ", Bed temp: " + str(self.bed_temp)

def find_key(key): # depreciated
	return{
	'X':'X',
	'Y':'Y',
	'Z':'Z',
	'F':'F',
	'E':'E',
	}.get(key,print("no action for " + key)) # Default case
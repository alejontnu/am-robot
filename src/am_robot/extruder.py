Hz = 1000

def feedrate():
	#calculate feedrate
return 0

def retraction_move(feedrate,retraction_distance):
	retraction_time = retraction_distance / feedrate
	control_steps = floor(Hz * retraction_time)
return 0

def recover_move():
	#calculate recover move
return 0

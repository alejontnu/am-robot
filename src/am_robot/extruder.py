

class ExtruderTool:
    def __init__(self,_tooltype,_ip):
        self.tooltype = _tooltype
        self.ip = _ip

Hz = 1000

def feedrate(F):
    '''
    Converts feedrate from mm/min to mm/s

    Parameters:
    -----------
    feedrate: int
    	rate of filament enxtrusion in mm/min

    Returns:
    -----------
    feedrate: float
        feedrate in mm/s
    '''
    feedrate = feedrate/60
    return feedrate

def retraction_move(feedrate,retraction_distance):
    retraction_time = retraction_distance / feedrate
    control_steps = floor(Hz * retraction_time)
    return 0

def recover_move():
    #calculate recover move
    print("Recover Move")
    return 0

def movement_time(e_target,e_current,feedrate):
    '''
    Calculates the time needed to extrude the change in filament position

    Parameters:
    -----------
    e_target: float
        target position of filament
    e_current: float
		current position of filament
    feedrate: int
    	rate of filament enxtrusion in mm/min

    Returns:
    -----------
    move_time: float
        float giving the time needed to extrude the desired amount of filament
    '''
    extruder_length = abs(e_target - e_current)
    move_time = extruder_length / feedrate
    return move_time
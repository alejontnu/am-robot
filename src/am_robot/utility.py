

class GeometryState: # G-commands, related to geometry, movement type, extrusion volume, etc
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

class MachineState: # M-commands, related to machine settings, temperature, motors, etc
    def __init__(self,_mcommand,_nozzle_temp,_bed_temp):
        self.mcommand = _mcommand
        self.nozzle_temp = _nozzle_temp
        self.bed_temp = _bed_temp

    def __str__(self):
        return 'M command: M' + str(self.mcommand) + ", Nozzle temp: " + str(self.nozzle_temp) + ", Bed temp: " + str(self.bed_temp)

def format_gcodeline(line):
    '''
    Helper function that converts a parsed Gcode line into a dict structure

    Parameters:
    -----------
    line: class
        the GcodeLine class from gcodeparser

    Returns:
    -----------
    Gline: dict
        dictionary containing gcode line command, parameters and comment
    '''
    comment = ''
    try:
        segment_comment = line.split(';')
        comment = segment_comment[1]
    except:
        pass
    else:
        line = segment_comment[0]
    finally:
        segmented = line.split()
        command_string = segmented.pop(0)
        command = (command_string[0],int(command_string[1:len(command_string)]))
        params = {}
        for element in segmented:
            params[element[0]] = float(element[1:len(element)])
        Gline = {
            "command": command,
            "params": params,
            "comment": comment
        }
    return Gline

def find_key(key): # depreciated
    return{
    'X':'X',
    'Y':'Y',
    'Z':'Z',
    'F':'F',
    'E':'E',
    }.get(key,print("no action for " + key)) # Default case
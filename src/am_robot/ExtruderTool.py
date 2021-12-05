import serial
import math

class ExtruderTool:
    '''
    Converts feedrate from mm/min to mm/s and passes it to extrusion controller

    Parameters:
    -----------
    feedrate: int
        rate of filament enxtrusion in mm/min

    Returns:
    -----------
    feedrate: float
        feedrate in mm/s
    '''
    def __init__(self,_tooltype,_port,_filament_width,_nozzle_diameter,_tool_transformation):
        self.tooltype = _tooltype
        self.port = _port
        self.filament_width = _filament_width
        self.nozzle_diameter = _nozzle_diameter
        self.T_tool = _tool_transformation

        self.ser = serial.Serial()
        self.ser.port = self.port

    def __str__(self):
        return "ToolType = "+str(self.tooltype)

    def set_feedrate(self,feedrate):
        self.ser.open()
        motor_frequency = feedrate_to_motor_frequency(feedrate)
        self.ser.write(b'X')# + 4 bytes of number
        self.close()

    def set_nozzletemp(self,temperature):
        self.ser.open()
        self.ser.write(b'H') # + 4 bytes of number
        self.close()

    def blink_led(self):
        self.ser.open()
        self.ser.write(b'B')
        self.close()

    def read_temperature(self):
        self.ser.open()
        self.ser.write(b'T')

        b = self.ser.read(5)

        letter = b[0]

        if letter == 84:
            [val] = struct.unpack('f',b[1:5])
            print(f"Temperature is {val} degree celsius")
        else:
            print("Value other than T read. Ignoring...")

        self.close()
        return 0

    def read_extrusion_speed(self):
        self.ser.open()
        self.ser.write(b'E')
        b = self.ser.read(5)

        letter = b[0]

        if letter == 69:
            [motor_frequency] = struct.unpack('f',b[1:5])
            feedrate = motor_frequency_to_feedrate(motor_frequency)
            print(f"Extrusion rate is {motor_frequency} mm/s")
        else:
            print("Value other than E read. Ignoring...")

        self.close()

    def enable_periodic_updates(self):
        self.ser.open()
        self.ser.write(b'Y')
        self.close()

    def disable_periodic_updates(self):
        self.ser.open()
        self.ser.write(b'N')
        self.close()

    def feedrate_to_motor_frequency(self,feedrate):
        motor_frequency = steps_per_mm_filament * feedrate
        return motor_frequency

    def motor_frequency_to_feedrate(self,motor_frequency):
        feedrate = motor_frequency / steps_per_mm_filament
        return feedrate

    def calculate_steps_per_mm(self):
        return motor_steps_per_revolution * micro_stepping * gear_ratio / (hobb_diameter_mm * math.pi)
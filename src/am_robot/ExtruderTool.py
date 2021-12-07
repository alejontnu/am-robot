import serial
import math
import struct

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

        self.motor_steps_per_revolution = 400
        self.micro_stepping = 16
        self.gear_ratio = 3 # From datasheet
        self.hobb_diameter_mm = 7.3 # From datasheet/manufacturer (effective and should be calibrated)

        self.steps_per_mm_filament = self.calculate_steps_per_mm()


    def __str__(self):
        return "ToolType = "+str(self.tooltype)

    def set_feedrate(self,feedrate):
        self.ser.open()
        motor_frequency = self.feedrate_to_motor_frequency(feedrate)
        packed = struct.pack('f',motor_frequency)
        self.ser.write(b'X'+packed)# + 4 bytes of number
        self.ser.close()

    def set_nozzletemp(self,temperature):
        self.ser.open()
        packed = struct.pack('f',temperature)
        self.ser.write(b'H'+packed) # + 4 bytes of number
        self.ser.close()

    def blink_led(self):
        self.ser.open()
        self.ser.write(b'B')
        self.ser.close()

    def set_fanspeed(self,speed):
        # not implemented
        x=1

    def disable_fan(self):
        # not implemented
        x=1

    def read_temperature(self):
        self.ser.open()
        self.ser.write(b'T')
        b = self.ser.read(5)
        self.ser.close()
        
        letter = b[0]

        if letter == 84:
            [temperature] = struct.unpack('f',b[1:5])
            print(f"Temperature is {temperature} degree celsius")
            return temperature
        else:
            print("Value other than T read. Ignoring...")

    def read_extrusion_speed(self):
        self.ser.open()
        self.ser.write(b'E')
        b = self.ser.read(5)
        self.ser.close()

        letter = b[0]

        if letter == 69:
            [motor_frequency] = struct.unpack('f',b[1:5])
            feedrate = self.motor_frequency_to_feedrate(motor_frequency)
            print(f"Extrusion rate is {feedrate} mm/s")
            return feedrate
        else:
            print("Value other than E read. Ignoring...")

    def enable_periodic_updates(self):
        self.ser.open()
        self.ser.write(b'Y')
        self.ser.close()

    def disable_periodic_updates(self):
        self.ser.open()
        self.ser.write(b'N')
        self.ser.close()

    def feedrate_to_motor_frequency(self,feedrate):
        motor_frequency = self.steps_per_mm_filament * feedrate
        return motor_frequency

    def motor_frequency_to_feedrate(self,motor_frequency):
        feedrate = motor_frequency / self.steps_per_mm_filament
        return feedrate

    def calculate_steps_per_mm(self):
        return self.motor_steps_per_revolution * self.micro_stepping * self.gear_ratio / (self.hobb_diameter_mm * math.pi)

    def convert_per_minute_to_per_second(self,value_per_minute):
        return value_per_minute/60

    def convert_per_second_to_per_minute(self,value_per_second):
        return value_per_second*60

    def calculate_difference(self,first_value,second_value):
        return second_value-first_value

    def calculate_delta_t(self,feedrate,delta_mm):
        return delta_mm/self.convert_per_minute_to_per_second(feedrate)

    def calculate_max_velocity(self,feedrate):
        cartesian_max_vel = 1700 #mm/s
        return self.convert_per_minute_to_per_second(feedrate)/cartesian_max_vel
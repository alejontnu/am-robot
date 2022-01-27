import serial
import math
import struct
import time

from am_robot.AbstractTool import AbstractTool

class ExtruderTool(AbstractTool):
    '''
    Converts feedrate from mm/min to mm/s and passes it to extrusion controller

    Attributes:
    -----------
    feedrate: int
        rate of filament enxtrusion in mm/min

    Methods:
    -----------
    feedrate: float
        feedrate in mm/s
    '''
    def __init__(self,port,tooltype,filament_width,nozzle_diameter,tool_transformation,skip_connection):
        '''
        Initialize extruder tool

        Input:
        -----

        Returns:
        -----

        '''
        super().__init__(port)


        self.tooltype = tooltype
        self.port = port
        self.filament_width = filament_width
        self.nozzle_diameter = nozzle_diameter
        self.T_tool = tool_transformation

        if not skip_connection:
            self.ser = serial.Serial(self.port)

        self.motor_steps_per_revolution = 400.0
        self.micro_stepping = 16.0
        self.gear_ratio = 3.0 # From datasheet
        self.hobb_diameter_mm = 7.68 # 7.3 # From datasheet/manufacturer (effective and should be calibrated)
        self.correcting_term = 1.0 # Change this to value after running extest.py and calculating true value

        self.steps_per_mm_filament = self.calculate_steps_per_mm()


    def disconnect(self):
        '''
        Disconnect the serial connection

        Input:
        -----

        Returns:
        -----

        '''
        self.ser.close()

    def __str__(self):
        return "ToolType = "+str(self.tooltype)

    def set_feedrate(self,feedrate):
        '''
        Set feedrate for the extruder

        Input:
        -----
        feedrate: float
            feedrate in mm/min

        Returns:
        ----

        '''
        feedrate = self.convert_per_minute_to_per_second(feedrate)
        motor_frequency = self.feedrate_to_motor_frequency(feedrate)
        packed = struct.pack('f',motor_frequency)
        self.ser.write(b'X'+packed)# + 4 bytes of number

    def set_nozzletemp(self,temperature):
        '''
        Set hotend temperature in Celsius

        Input:
        -----
        temperature: float
            temperature in degree celsius

        Returns:
        ----

        '''
        packed = struct.pack('f',temperature)
        self.ser.write(b'H'+packed) # + 4 bytes of number

    def blink_led(self):
        '''
        Blink the arduino led once

        Input:
        -----

        Returns:
        -----

        '''
        self.ser.write(b'B')

    def set_fanspeed(self,speed):
        '''
        Sets the fanspeed

        Input:
        -----

        Returns:
        -----

        '''
        print(" Fan speed not implemented")

    def disable_fan(self):
        '''
        Turns off fans

        Input:
        -----

        Returns:
        -----

        '''
        print("Disable fan not implemented")

    def read_temperature(self):
        '''
        Read and return temperature of hotend in Celsius

        Input:
        -----

        Returns:
        -----
        temperature: float
            Temperature of hotend in Celsius

        '''
        read_temp = True
        while read_temp:
            #self.ser.write(b'T')
            b = self.ser.read(5)

            letter = b[0]

            if letter == 84:
                [temperature] = struct.unpack('f',b[1:5])
                print(f"Temperature is {temperature} degree celsius")
                read_temp = False
                return temperature
            else:
                print("Value other than T read. Ignoring...")
                time.sleep(0.1)

    def read_extrusion_speed(self):
        '''
        Read and return extrusion rate in mm/s

        Input:
        -----

        Returns:
        -----
        feedrate: float
            Feedrate of extrusion process in mm/s

        '''
        read_extrusion = True
        while read_extrusion:
            b = self.ser.read(5)

            letter = b[0]

            if letter == 69:
                [motor_frequency] = struct.unpack('f',b[1:5])
                feedrate = self.motor_frequency_to_feedrate(motor_frequency)
                print(f"Extrusion rate is {feedrate} mm/s")
                read_extrusion = False
                return feedrate
            else:
                print("Value other than E read. Ignoring...")
                time.sleep(0.1)

    def enable_periodic_updates(self):
        '''
        Enable periodic updates from arduino. Updates include temperature readout and extrusion rate
        '''
        self.ser.write(b'Y')

    def disable_periodic_updates(self):
        '''
        Disable periodic updates from arduino.
        '''
        self.ser.write(b'N')

    def feedrate_to_motor_frequency(self,feedrate):
        '''
        Converts feedrate mm/s to motor frequency Hz

        Input:
        -----
        feedrate: float
            Feedrate of extrusion process in mm/s

        Returns:
        -----
        motor_frequency: float
            Motor frequency in Hertz based on steps per mm filament

        '''
        motor_frequency = self.steps_per_mm_filament * feedrate
        return motor_frequency

    def motor_frequency_to_feedrate(self,motor_frequency):
        '''
        Converts motor frequency Hz to feedrate mm/s

        Input:
        -----
        motor_frequency: float
            motor frequency in Hertz

        Returns:
        -----
        feedrate: float
            Feedrate in mm/s based on steps per mm filament

        '''
        feedrate = motor_frequency / self.steps_per_mm_filament
        return feedrate

    def calculate_steps_per_mm(self):
        '''
        Calculates the stepper motor steps per mm filament

        Input:
        -----

        Returns:
        -----
        steps_per_mm: float
            Number of stepper motor 'steps' per mm input filament

        '''
        return (self.motor_steps_per_revolution * self.micro_stepping * self.gear_ratio) / (self.hobb_diameter_mm * math.pi)

    def convert_per_minute_to_per_second(self,value_per_minute):
        '''
        Devides by 60
        '''
        return value_per_minute/60.0

    def convert_per_second_to_per_minute(self,value_per_second):
        '''
        Multiplies by 60
        '''
        return value_per_second*60.0

    def calculate_difference(self,first_value,second_value):
        '''
        Returns the difference of two values
        '''
        return second_value-first_value

    def calculate_delta_t(self,first_value,second_value,feedrate):
        '''
        Returns the time taken to process the difference between two distances
        '''
        return self.calculate_difference(first_value,second_value)/self.convert_per_minute_to_per_second(feedrate)

    def calculate_max_rel_velocity(self,feedrate,robot_vel_constraint):
        '''
        Returns the relative velocity ratio between robot maximum and desired process speed
        '''
        return self.convert_per_minute_to_per_second(feedrate)/robot_vel_constraint
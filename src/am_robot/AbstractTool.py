class AbstractTool():
    def __init__(self,port):
        self.port = port


    def disconnect(self):
        print("Function 'disconnect()' missing in Specific tool Class, please implement...")

    def set_feedrate(self,feedrate):
        print("Function 'disconnect()' missing in Specific tool Class, please implement...")

    def set_nozzletemp(self,temperature):
        print("Function 'set_feedrate(temperature)' missing in Specific tool Class, please implement...")

    def blink_led(self):
        print("Function 'blink_led()' missing in Specific tool Class, please implement...")

    def set_fanspeed(self,speed):
        print("Function 'set_fanspeed(speed)' missing in Specific tool Class, please implement...")

    def disable_fan(self):
        print("Function 'disable_fan()' missing in Specific tool Class, please implement...")

    def read_temperature(self):
        print("Function 'read_temperature()' missing in Specific tool Class, please implement...")

    def read_extrusion_speed(self):
        print("Function 'read_extrusion_speed()' missing in Specific tool Class, please implement...")

    def enable_periodic_updates(self):
        print("Function 'enable_periodic_updates()' missing in Specific tool Class, please implement...")

    def disable_periodic_updates(self):
        print("Function 'disable_periodic_updates()' missing in Specific tool Class, please implement...")

    def feedrate_to_motor_frequency(self,feedrate):
        print("Function 'feedrate_to_motor_frequency(feedrate)' missing in Specific tool Class, please implement...")

    def motor_frequency_to_feedrate(self,motor_frequency):
        print("Function 'motor_frequency_to_feedrate(motor_frequency)' missing in Specific tool Class, please implement...")

    def calculate_steps_per_mm(self):
        print("Function 'calculate_steps_per_mm()' missing in Specific tool Class, please implement...")

    def convert_per_minute_to_per_second(self,value_per_minute):
        print("Function 'convert_per_minute_to_per_second(value_per_minute)' missing in Specific tool Class, please implement...")

    def convert_per_second_to_per_minute(self,value_per_second):
        print("Function 'convert_per_second_to_per_minute(value_per_second)' missing in Specific tool Class, please implement...")

    def calculate_difference(self,first_value,second_value):
        print("Function 'calculate_difference(first_value,second_value)' missing in Specific tool Class, please implement...")

    def calculate_delta_t(self,first_value,second_value,feedrate):
        print("Function 'calculate_delta_t(first_value,second_value,feedrate)' missing in Specific tool Class, please implement...")

    def calculate_max_rel_velocity(self,feedrate,robot_vel_constraint):
        print("Function 'calculate_max_rel_velocity(feedrate,robot_vel_constraint)' missing in Specific tool Class, please implement...")


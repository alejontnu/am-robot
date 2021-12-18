import serial
import struct
import time
import argparse
import math


def main():

	E = 69
	T = 84
	X = 88
	H = 72
	B = 66



	#ser = serial.Serial('/dev/ttyS5',9600,timeout=1,parity='N',rtscts=1)
	#ser = serial.Serial('/dev/ttyS5')#,9600,timeout=1,parity='N',rtscts=1)
	ser = serial.Serial('/dev/ttyUSB0')


	read_serial(ser)
	set_nozzletemp(200.0,ser)
	set_feedrate(feedrate_to_motor_frequency(0),ser)

	for i in range(10):
		read_serial(ser)
		time.sleep(1)

	ser.close()

def read_serial(ser):
	ser.write(b'T')

	b = ser.read(10)

	letter1 = b[0]
	letter2 = b[5]

	if letter1 == 84:
		[val] = struct.unpack('f',b[1:5])
		print(f"Temperature is {val} degree celsius")
		if letter2 == 69:
			[val] = struct.unpack('f',b[6:10])
			print(f"Extrusion rate is {val} Hz")
	elif letter1 == 69:
		[val] = struct.unpack('f',b[1:5])
		print(f"Extrusion rate is {val} Hz")
		if letter2 == 84:
			[val] = struct.unpack('f',b[6:10])
			print(f"Temperature is {val} degree celsius")
	else:
		print("Value other than T or E read. Ignoring...")

def set_feedrate(feedrate,ser):
	packed = struct.pack('f',feedrate)
	ser.write(b'X'+packed)# + 4 bytes of number

def set_nozzletemp(temperature,ser):
	packed = struct.pack('f',temperature)
	ser.write(b'H'+packed) # + 4 bytes of number

def feedrate_to_motor_frequency(feedrate):
    '''
    Converts feedrate mm/s to motor frequency Hz
    '''
    motor_frequency = calculate_steps_per_mm() * feedrate
    return motor_frequency

def motor_frequency_to_feedrate(motor_frequency):
    '''
    Converts motor frequency Hz to feedrate mm/s
    '''
    feedrate = motor_frequency / calculate_steps_per_mm()

    return feedrate

def calculate_steps_per_mm():
    '''
    Calculates the stepper motor steps per mm filament
    '''
    motor_steps_per_revolution = 400.0
    micro_stepping = 16.0
    gear_ratio = 3.0 # From datasheet
    hobb_diameter_mm = 7.68 # 7.3
    return (motor_steps_per_revolution * micro_stepping * gear_ratio) / (hobb_diameter_mm * math.pi)


if __name__ == '__main__':
	main()
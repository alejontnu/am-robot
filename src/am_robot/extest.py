import serial
import struct
import time
import argparse


def main():

	E = 69
	T = 84
	X = 88
	H = 72
	B = 66

	#ser = serial.Serial('/dev/ttyS5',9600,timeout=1,parity='N',rtscts=1)
	ser = serial.Serial('/dev/ttyS5')#,9600,timeout=1,parity='N',rtscts=1)

	read_serial(ser)
	set_nozzletemp(50,ser)

def read_serial(ser):
	ser.write(b'E')

	b = ser.read(5)

	ser.close()

	letter = b[0]
	print(f"Letter: {letter}")

	if letter == 84:
		[val] = struct.unpack('f',b[1:5])
		print(f"Temperature is {val} degree celsius")
	elif letter == 69:
		[val] = struct.unpack('f',b[1:5])
		print(f"Extrusion rate is {val} Hz")
	else:
		print("Value other than T or E read. Ignoring...")

def set_feedrate(feedrate,ser):
	ser.open()
	packed = struct.pack('b',feedrate)
	ser.write(b'X'+packed)# + 4 bytes of number
	ser.close()

def set_nozzletemp(temperature,ser):
	ser.open()
	packed = struct.pack('b',temperature)
	print(packed)
	ser.write(b'H') # + 4 bytes of number
	ser.write(packed)
	ser.close()

if __name__ == '__main__':
	main()
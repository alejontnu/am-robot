import serial
import struct
import time
import argparse


def main():

	parser = argparse.ArgumentParser(add_help=True)

	parser.add_argument('--letter',default='B')

	args = parser.parse_args()

	E = 69
	T = 84
	X = 88
	H = 72
	B = 66


	#ser = serial.Serial('/dev/ttyS5',9600,timeout=1,parity='N',rtscts=1)
	ser = serial.Serial('/dev/ttyS5')#,9600,timeout=1,parity='N',rtscts=1)

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
		print(f"Extrusion rate is {val} mm/s")
	else:
		print("Value other than T or E read. Ignoring...")

if __name__ == '__main__':
	main()
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
	set_nozzletemp(20.4761905670166,ser)
	set_feedrate(20,ser)

def read_serial(ser):
	ser.write(b'T')

	b = ser.read(5)
	print(b)
	ser.close()

	letter = b[0]
	print(f"Letter: {letter}")

	if letter == 84:
		print(b[1:5])
		[val] = struct.unpack('f',b[1:5])
		print(f"Temperature is {val} degree celsius")
	elif letter == 69:
		[val] = struct.unpack('f',b[1:5])
		print(f"Extrusion rate is {val} Hz")
	else:
		print("Value other than T or E read. Ignoring...")

def set_feedrate(feedrate,ser):
	ser.open()
	packed = struct.pack('f',feedrate)
	print(feedrate)
	print(packed)
	ser.write(b'X'+packed)# + 4 bytes of number
	print(b'X'+packed)
	ser.close()

def set_nozzletemp(temperature,ser):
	ser.open()
	print(temperature)
	packed = struct.pack('f',temperature)
	print(packed)
	print(b'H')
	ser.write(b'H'+packed) # + 4 bytes of number
	ser.close()

if __name__ == '__main__':
	main()
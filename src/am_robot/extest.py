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
	set_nozzletemp(40,ser)
	set_feedrate(50,ser)
	for i in range(10):
		read_serial(ser)
		time.sleep(1)

	ser.close()

def read_serial(ser):
	ser.write(b'T')

	b = ser.read(10)
	print(b)


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
	print(feedrate)
	print(packed)
	ser.write(b'X'+packed)# + 4 bytes of number
	print(b'X'+packed)


def set_nozzletemp(temperature,ser):

	print(temperature)
	packed = struct.pack('f',temperature)
	print(packed)
	print(b'H')
	ser.write(b'H'+packed) # + 4 bytes of number


if __name__ == '__main__':
	main()
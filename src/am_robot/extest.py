import serial
import struct
import time
import argparse
import math


def main():
    '''
    For calibrating extruder extrusion rate. Setting rate to 10mm/s and extruding for 100s with the expected extrusion length to be 1000mm. Ratio found can be used to find a correctiong term.
    '''
    parser = argparse.ArgumentParser(
        formatter_class=argparse.MetavarTypeHelpFormatter,
        description=('''To calibrate extrusion rate'''),
        add_help=True)

    parser.add_argument('--port', default='/dev/ttyUSB0', type=str, help='Serial connection of the tool used')
    parser.add_argument('--test', default='extrusion',type=str,help='Type of test')

    args = parser.parse_args()

    E = 69
    T = 84
    X = 88
    H = 72
    B = 66

    ser = serial.Serial(args.port)

    feedrate = 600

    set_nozzletemp(200.0,ser)

    while True:
        temp = read_serial(ser)
        if temp > 200.0:
            break

    print("Starting extrusion at 10mm/s")
    set_feedrate(feedrate_to_motor_frequency(0.0),ser)

    set_feedrate(feedrate_to_motor_frequency(feedrate/60.0),ser)

    print("Sleeping for 10 seconds")
    time.sleep(10)

    print("Done! Stopping extrusion...")
    set_feedrate(feedrate_to_motor_frequency(0.0),ser)

    set_nozzletemp(0.0,ser)

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
            [val2] = struct.unpack('f',b[6:10])
            print(f"Extrusion rate is {val2} Hz")
        return val
    elif letter1 == 69:
        [val] = struct.unpack('f',b[1:5])
        print(f"Extrusion rate is {val} Hz")
        if letter2 == 84:
            [val2] = struct.unpack('f',b[6:10])
            print(f"Temperature is {val2} degree celsius")
            return val2
    else:
        print("Value other than T or E read. Ignoring...")


def set_feedrate(feedrate,ser):
    packed = struct.pack('f',feedrate)
    ser.write(b'X'+packed)  # + 4 bytes of number


def set_nozzletemp(temperature,ser):
    packed = struct.pack('f',temperature)
    ser.write(b'H'+packed)  # + 4 bytes of number


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
    gear_ratio = 3.0  # From datasheet
    hobb_diameter_mm = 7.68  # 7.3
    return (motor_steps_per_revolution * micro_stepping * gear_ratio) / (hobb_diameter_mm * math.pi)


if __name__ == '__main__':
    main()

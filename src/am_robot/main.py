import os
# import am_robot
from gcodeparser import GcodeParser

# TODO - filename from argparse

def main():
    filename = 'circles.gcode'
    folder = 'data'

    print(os.getcwd())

    fullpath = os.path.join('.',folder,filename)
    print(fullpath)
    print(os.listdir('data'))

    # for root, dirs, files in os.walk(".", topdown=True):
    #     print('start')
    #     for dir in dirs:
    #         print(os.path.join(root, dir))
    #     print('stop-start')
    #     for file in files:
    #         print(os.path.join(root, file))
    #     print('stop')



    # print(am_robot.__file__.split)

    with open(fullpath,'r') as file:
        gcode = file.read()


    parsed_gcode = GcodeParser(gcode)
    for line in parsed_gcode.lines:
        print(line)

if __name__ == '__main__':
    main()
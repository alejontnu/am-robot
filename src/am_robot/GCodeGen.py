import argparse
from math import floor
import os

class GCodeGen():
    def __init__(self,function,filename,time):
        self.equation_of_motion = function
        self.filename = filename
        self.time = time
        self.initial_layer_height = 0.6
        self.layer_height = 0.4
        self.D_nozzle = 0.8
        self.D_filament = 2.85
        self.params = ['X','Y','Z','F','E']

    def find_path(self):
        filename, extension = os.path.splitext(self.filename)
        if extension == '':
            print("No file extension given, assuming '.gcode'")
            extension = '.gcode'
        elif extension.lower() == '.gcode':
            filename, extension = os.path.splitext(self.filename)
        else:
            print("File extension not empty or .gcode")
            input("finding for '.gcode' instead. Enter to continue...")
            extension = '.gcode'

        cwd = os.getcwd()
        self.fullpath = os.path.join(cwd, 'data',filename+extension)

    def create_file(self):
        self.find_path()
        with open(self.fullpath,'w') as file:
            file.write('; G-code generated by GCodeGen.py')
        
    def save_to_file(self,string):
        with open(self.fullpath,'a') as file:
            file.write('\n')
            file.write(string)

    def calculate_next_point(self,t):
        self.next_command = 'G1'
        self.next_point = [3.0,4.0,2.0,1800]
        self.next_params = ['X','Y','Z','F']
        self.next_comment = ''

    def format_string(self):
        string = ''
        string = string + self.next_command
        if self.next_command == 'G1':
            for i in range(len(self.next_params)):
                string = string + ' ' + self.next_params[i] + str(self.next_point[i])
        if self.next_comment != '':
            string = string + ' ; ' + self.next_comment
        self.line = string



def main():
    '''
    Input:
    ----
    --func <list of strings>
        A list of elements when each element corresponds to the contineous time equation of a parameter. Default: ['x*t','y*t','0.1*z*t']
    '''
    parser = argparse.ArgumentParser(formatter_class=argparse.MetavarTypeHelpFormatter,
        description=('''To generate a basic G-Code file from a continuous time function''')
        ,epilog='This is still under development',
        add_help=True)

    parser.add_argument('--func', default=['cos(t)','sin(t)','0.5*t'], type=list, help='Contineous time motion function as a string')
    parser.add_argument('--file',default='motion.gcode',type=str,help='String name of outut file')
    parser.add_argument('--time',default=10.0,type=float,help='Time frame of movement')
    parser.add_argument('--dt',default=0.1,type=float,help='The size of a time step')

    args = parser.parse_args()

    generator = GCodeGen(args.func,args.file,args.time)
    generator.create_file()

    for i in range(floor(args.time/args.dt)):
        generator.calculate_next_point(i)
        generator.format_string()
        generator.save_to_file(generator.line)



if __name__ == '__main__':
    main()
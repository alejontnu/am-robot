import time

class GCodeCommands():
    '''
    Collection of G-code commands (M and G). 

    Attributes:
    ----

    Methods:
    ----

    '''
    def __init__(self):
        pass

    def default(self):
        print(f"No method for command: {self.command} in GCodeCommands class")
        input("Paused... Press enter to continue...")

    ''' M-command methods '''

    def M82(self):
        print("E absolute - Abort if robot uses relative")
        self.E_positioning = 'abs'

    def M83(self):
        print("E Relative - Abort if robot uses absolute")
        self.E_positioning = 'rel'

    def M84(self):
        print("Disable motors - Not implemented")

    def M104(self):
        print("Setting hotend reference temperature")
        self.tool.set_nozzletemp(self.read_param(self.interval[0],'S'))

    def M105(self):
        print("Getting nozzle temperature reading")
        print(self.tool.read_temperature())

    def M106(self):
        print("Set fan speed - Not implemented")

    def M107(self):
        print("Fan off - Not implemented")

    def M109(self):
        print("Setting and waiting for hotend temperature")
        if self.read_param(self.interval[0],'S') != False:
            self.tool.set_nozzletemp(self.read_param(self.interval[0],'S'))
            while self.tool.read_temperature() < self.read_param(self.interval[0],'S')-5:
                print(".")
                time.sleep(1)
        elif self.read_param(self.interval[0],'R') != False:
            self.tool.set_nozzletemp(self.read_param(self.interval[0],'R'))
            while self.tool.read_temperature() < self.read_param(self.interval[0],'R')-5 or self.tool.read_nozzletemp() > self.read_param(interval[0],'R')+5:
                print(".")
                time.sleep(1)
        else:
            self.tool.set_nozzletemp(0)
            while self.tool.read_temperature() > 35: #assumed high ambient temperature
                print(".")
                time.sleep(1)

    def M140(self):
        print("Set bed temperature")

    ''' G-command methods '''

    def G0(self):
        print("linear non-extrusion move")

    def G1(self):
        print("linear extrusion move")

    def G20(self):
        print("set units to inches")
        self.units = 'inch'

    def G21(self):
        print("set units to millimeters")
        self.units = 'mm'

    def G28(self):
        print("Auto home")

    def G29(self):
        print("Bed leveling")

    def G90(self):
        print("Use absolute coordinates")
        self.E_positioning = 'abs'
        self.X_positioning = 'abs'
        self.Y_positioning = 'abs'
        self.Z_positioning = 'abs'

    def G91(self):
        print("Use relative coordinates")
        self.E_positioning = 'rel'
        self.X_positioning = 'rel'
        self.Y_positioning = 'rel'
        self.Z_positioning = 'rel'

    def G92(self):
        print("Reset extreuder/all distances")
        for key in self.gcodelines[self.interval[0]].params:
            self.__dict__[key] = self.read_param(self.interval[0],key)


# Call commands like so:
def main():
    gcc = GCodeCommands()
    command = 'G1'
    gcc.command = command
    getattr(gcc,command,getattr(gcc,'default'))() # <---


if __name__ == '__main__':
    main()
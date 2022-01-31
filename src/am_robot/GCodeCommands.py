import time
import numpy as np

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
        print("Disable motors - Only disables extruder motor")
        self.tool.set_feedrate(0.0)

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
            temp_ref = self.read_param(self.interval[0],'S')
            self.tool.set_nozzletemp(temp_ref)
            while self.tool.read_temperature() < (temp_ref-5.0):
                print(".")
        elif self.read_param(self.interval[0],'R') != False:
            self.tool.set_nozzletemp(self.read_param(self.interval[0],'R'))
            while self.tool.read_temperature() < self.read_param(self.interval[0],'R')-5.0 or self.tool.read_nozzletemp() > self.read_param(self.interval[0],'R')+5.0:
                print(".")
        else:
            self.tool.set_nozzletemp(0)
            while self.tool.read_temperature() > 35.0: #assumed high ambient temperature
                print(".")
        time.sleep(3)

    def M140(self):
        print("Set bed temperature - Not implemented")

    ''' G-command methods '''

    def G0(self):
        #if self.read_param(interval[0],'X') != False or self.read_param(interval[0],'Y') != False or self.read_param(interval[0],'Z') != False:
        if self.read_param(self.interval[0],'X') != False and self.read_param(self.interval[0],'Y') != False:
            motion = self.make_path(self.interval,0.01)
            #self.robot.velocity_rel = self.tool.calculate_max_rel_velocity(self.F,self.robot.max_cart_vel)

            input("Press enter to start non-extrusion move...")
            self.robot.execute_move(frame=self.robot.tool_frame,motion=motion)

    def G1(self):
        if (self.read_param(self.interval[0],'X') != False) or (self.read_param(self.interval[0],'Y') != False) or (self.read_param(self.interval[0],'Z') != False):

            # Make path trajectory
            motion = self.make_path(self.interval,0.01) # PathMotion gives smooth movement compared to WayPointMovement

            # set dynamic rel and relative max velocity based on feedrate
            rel_velocity = self.tool.calculate_max_rel_velocity(self.F,self.robot.max_cart_vel)
            self.robot.set_velocity_rel(rel_velocity)

            # set extrusion speed if needed. Some slicers use G1 for non extrusion moves...
            if self.read_param(self.interval[0],'E') != False:
                self.tool.set_feedrate(self.F/40.0)
            # feed path motion to robot and move using a separate thread
            thread = self.robot.execute_threaded_move(frame=self.robot.tool_frame,motion=motion) # Just starts move in a thread with some initialization

            # Wait here for path motion to finish and join the thread
            thread.join()

            # Thread done aka move done aka stop extrusion immidiately
            self.tool.set_feedrate(0)
            self.robot.recover_from_errors()

        # Some slicers use G1 even for non extrusion moves... Example retraction of filament
        elif self.read_param(self.interval[0],'E') != False:
            # Target extrusion distance and time elapsed at given feedrate
            target_E = self.read_param(self.interval[0],'E')
            sleep_time = self.tool.calculate_delta_t(target_E,self.E,self.F)

            # set retraction/un-retraction feedrate
            self.tool.set_feedrate(np.sign(sleep_time)*self.F/40.0)
            # Sleep
            time.sleep(abs(sleep_time))
            # Stop retraction/un-retraction
            self.tool.set_feedrate(0)

        self.set_params(self.interval[1])

    def G2(self):
        print("Clockwise arc/circle extrusion move - Not implemented (Robot specific)")

    def G3(self):
        print("Counter-clockwise arc/circle extrusion move - Not implemented (Robot specific)")

    def G10(self):
        print("Retraction move - Not implemented")

    def G11(self):
        print("Recover move (after retraction) - Not implemented")

    def G20(self):
        print("set units to inches")
        self.units = 'inch'

    def G21(self):
        print("set units to millimeters")
        self.units = 'mm'

    def G28(self):
        print("Auto home")
        x = 0.0
        y = 0.0
        z = 0.0

        # is params, move slightly above highest print height
        nr_keys = 0
        for key in self.gcodelines[self.interval[0]].params:
            nr_keys = nr_keys + 1


        if nr_keys == 0:
            # if not params move to default 0,0,0 start position
            self.move_to_point(x,y,z)
        else:
            self.move_to_point(self.Xmax[0],self.Ymax[0],self.Zmax[1]+0.02)

    def G29(self):
        print("Bed leveling - Not implemented (done pre-emptively)")

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
        print("Reset extruder/all distances")
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
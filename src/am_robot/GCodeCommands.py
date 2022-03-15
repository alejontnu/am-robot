import time
import numpy as np
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
        #print(f"No method for command: {self.command} in GCodeCommands class")
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

    def M1044(self):
        print("Setting hotend reference temperature")
        self.tool.set_nozzletemp(self.read_param(self.interval[0],'S'))

    def M105(self):
        print("Getting nozzle temperature reading")
        print(self.tool.read_temperature())

    def M106(self):
        print("Set fan speed - Not implemented")

    def M107(self):
        print("Fan off - Not implemented")

    def M1099(self):
        print("Setting and waiting for hotend temperature")
        if self.read_param(self.interval[0],'S') is not False:
            temp_ref = self.read_param(self.interval[0],'S')
            self.tool.set_nozzletemp(temp_ref)
            while True:  #self.tool.read_temperature() < (temp_ref-5.0):
                self.tool.read_temperature()
                pass
        elif self.read_param(self.interval[0],'R') is not False:
            self.tool.set_nozzletemp(self.read_param(self.interval[0],'R'))
            while self.tool.read_temperature() < self.read_param(self.interval[0],'R')-5.0 or self.tool.read_nozzletemp() > self.read_param(self.interval[0],'R')+5.0:
                pass
        else:
            self.tool.set_nozzletemp(0)
            while self.tool.read_temperature() > 35.0:  # assumed high ambient temperature
                pass
        # Giving time for temperature to stabilize
        time.sleep(3)

    def M140(self):
        print("Set bed temperature - Not implemented")

    ''' G-command methods '''

    def G0(self):
        # if self.read_param(interval[0],'X') is not False or self.read_param(interval[0],'Y') is not False or self.read_param(interval[0],'Z') is not False:
        if self.read_param(self.interval[0],'X') is not False and self.read_param(self.interval[0],'Y') is not False:
            motion = self.make_path(self.interval,0.01)
            # self.robot.velocity_rel = self.tool.calculate_max_rel_velocity(self.F,self.robot.max_cart_vel)

            input("Press enter to start non-extrusion move...")
            self.robot.execute_move(frame=self.robot.tool_frame,motion=motion)

    def G1(self):
        if (self.read_param(self.interval[0],'X') is not False) or (self.read_param(self.interval[0],'Y') is not False) or (self.read_param(self.interval[0],'Z') is not False):

            # set dynamic rel and relative max velocity based on feedrate
            rel_velocity = self.tool.calculate_max_rel_velocity(self.F,self.robot.max_cart_vel*1000)
            self.robot.set_velocity_rel(rel_velocity)

            # Make path motion trajectory, the additional path is the more fancy one that is not yet implemented in Robot.move() but used for its time parametrization states
            path_motion, path = self.make_path(self.interval,0.01)  # PathMotion gives smooth movement compared to WayPointMovement

            # set extrusion speed if needed. Some slicers use G1 for non extrusion moves...
            if self.read_param(self.interval[0],'E') is not False:
                # parametrize the path to get states
                timestep = 0.01
                vel_rels = [0.395*self.robot.robot.velocity_rel*self.robot.max_cart_vel]*7  # *7 for a 7 element list. 0.395 because ???
                accel_rels = [self.robot.robot.acceleration_rel*self.robot.max_cart_acc]*7
                jerk_rels = [self.robot.robot.jerk_rel*self.robot.max_cart_jerk]*7

                # print(f"vel_rel: {vel_rels}")
                # print(f"accel_rel: {accel_rels}")
                # print(f"jerk_rel: {jerk_rels}")

                t_list, s_list, v_list, a_list, j_list = self.robot.parametrize_path(path,timestep,vel_rels,accel_rels,jerk_rels)

                path_time = len(t_list) * timestep

                # Find mm filament per mm robot movement
                mm_filament_per_mm_distance = self.path_extrusion / (path.length * 1000)

                # Simple average
                # path_time = len(t_list) * 0.01
                # average_extrusion_velocity = (self.path_extrusion / path_time) * 60
                # self.tool.set_feedrate(average_extrusion_velocity)

                # Prot the velocity curve
                # self.plot_cart_path(t_list, s_list, v_list, a_list, j_list)

            start = time.perf_counter()
            # feed path motion to robot and move using a separate thread
            thread = self.robot.execute_threaded_move(frame=self.robot.tool_frame,motion=path_motion)  # Just starts move in a thread with some initialization

            if self.read_param(self.interval[0],'E') is not False:
                for velocity in v_list:
                    if not thread.is_alive():
                        self.tool.set_feedrate(0.0)
                        break
                    else:
                        feedrate_profile_per_s = mm_filament_per_mm_distance * velocity * 1000
                        feedrate_profile_per_min = feedrate_profile_per_s * 60

                        # actual_avg_time = path.length /
                        print(f"Robot rel_vel: {rel_velocity}")
                        print(f"Time param velocity: {velocity}")
                        print(f"Feedrate: {feedrate_profile_per_min}")
                        print(f"Feedratef rom gcode: {self.F}")
                        print(f"Move time: {path_time}")
                        print(f"Path length: {path.length}")
                        print(f"Path distance filament: {self.path_extrusion}")
                        # self.tool.set_feedrate(feedrate_profile_per_min)
                        time.sleep(timestep)

            # Wait here for path motion to finish and join the thread
            thread.join()
            end = time.perf_counter()
            print(f"Move time: {end-start}s")

            # Thread done aka move done aka stop extrusion immidiately
            self.tool.set_feedrate(0.0)
            self.robot.recover_from_errors()

        # Some slicers use G1 even for non extrusion moves... Example retraction of filament
        elif self.read_param(self.interval[0],'E') is not False:
            # Target extrusion distance and time elapsed at given feedrate
            target_E = self.read_param(self.interval[0],'E')
            sleep_time = self.tool.calculate_delta_t(target_E,self.E,self.F)

            # set retraction/un-retraction feedrate
            self.tool.set_feedrate(np.sign(sleep_time)*self.F)

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
    getattr(gcc,command,getattr(gcc,'default'))()  # <---


if __name__ == '__main__':
    main()

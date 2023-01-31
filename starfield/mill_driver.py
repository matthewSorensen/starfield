import numpy as np
from pewpew.planner import MotionPlanner, KinematicLimits
from pewpew import MachineConnection, find_usbtty, bind_halt_signal
from pewpew.definitions import StatusFlag, OverrideMessage
from starfield.driver import MachineDriver
import time

steps_per_inch = np.array([8000,8000,8000,8000])

a = np.array([5.0,5.0,4.0,5.0]) / 5
v = np.array([30.0,30.0,10.0,10.0]) / 60.0


location = (0,0,0)
ones = np.ones(4)
plan = MotionPlanner(KinematicLimits(v, a, 5, 0.05), steps_per_inch, np.zeros(4))


def get_planner(connection):
    status = connection.status()
    while status is None:
        time.sleep(0.25)
        status = connection.status()
        
    position = (np.array(status.position) / steps_per_inch)
    return MotionPlanner(KinematicLimits(v, a, 5, 0.05), steps_per_inch, position)


class PewPewDriver(MachineDriver):
    """A machine driver for a 4-axis instance of PewPew - lightwight firmware for galvo-scanned
    fiber lasers and other general CNC tasks. Available at https://github.com/matthewSorensen/pewpew """

    def __init__(self, connection, planner):
        self.connection = connection
        self.planner = planner
        super().__init__()

    def block_for_status(self):
        status = self.connection.status()
        while status is None:
            time.sleep(0.25)
            status = self.connection.status()
        return status
        
    def move_to(self, position, velocity = None, block = False):
        destination = np.zeros(4)
        destination[0:3] = position

        self.connection.buffered_messages(list(self.planner.plan_moves([destination], velocity)))
        self.connection.busy.wait()

        if block:
            self.connection.wait_until_idle()
            
    def finished(self):
        flag = self.block_for_status().status_flag
        return flag == StatusFlag.IDLE or flag == StatusFlag.HALT

    def wait(self):
        self.connection.wait_until_idle()

    def get_current_position(self):
        status = self.block_for_status()
        return (np.array(status.position) / self.planner.microsteps)[0:3]
        
    def halt(self):

        #
        #    machine.realtime_message(OverrideMessage(1/32, 0.001))

        # Send the override message
        # wait until it's stopped
        # read back the current position and update the planner
        
        pass

import time

class MachineDriver:
    """A generic interface for interfacing to various motion control systems"""

    def __init__(self):
        pass

    def move_to(self, position, velocity = None, block = False):
        pass

    def finished(self):
        """ Query if the current motion is finished. Returns False if the controller is still moving, True otherwise. 
        May block for a short time to poll a status. """
        return False

    def wait(self):
        while not self.finished():
            time.sleep(0.25)
    
    def get_current_position(self):
        pass

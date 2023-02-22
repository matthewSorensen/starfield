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

    def home_z_axis(self):
        """ Reference the machine along the camera's optical axis - will not be called with a deployed camera. """
        pass

    def stow_camera(self):
        pass

    def deploy_camera(self):
        pass

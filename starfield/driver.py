import time
import importlib
import numpy as np

class MachineDriver:
    """A generic interface for interfacing to various motion control systems"""

    REALTIME_POSITION = False
    
    def __init__(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, type, value, tb):
        pass
    
    def move_to(self, position, velocity = None, block = False):
        pass

    def finished(self):
        """ Query if the current motion is finished. Returns False if the controller is still moving, True otherwise. 
        May block for a short time to poll a status. """
        return True

    def wait(self):
        while not self.finished():
            time.sleep(0.25)
    
    def get_current_position(self):
        return np.zeros(3)

    def home_z_axis(self):
        """ Reference the machine along the camera's optical axis - will not be called with a deployed camera. """
        pass

    def stow_camera(self):
        pass

    def deploy_camera(self):
        pass


def load_driver(path):
    path = path.split('.')
    module,clss = '.'.join(path[:-1]), path[-1]

    try:
        mod = importlib.import_module(module)
    except:
        print(f"Unable to find module {module}")
        exit(-1)

    if clss not in mod.__dict__:
        print(f"Unable to find driver {clss} in module {module}")
        exit(-1)
        
    return mod.__dict__[clss]

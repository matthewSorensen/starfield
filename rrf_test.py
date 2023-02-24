


import serial
from serial.threaded import LineReader, ReaderThread
import sys
import time
import numpy as np
from threading import Event
import json

from starfield.driver import MachineDriver

class RRF3Connection(LineReader):
    TERMINATOR = b'\n'

    def connection_made(self, transport):
        super(RRF3Connection, self).connection_made(transport)
        self.idle = Event()
        self.position = None
        self.position_event = Event()
        
    def ask_status(self):
        self.write_line('M409 K"state.status"\n')

    def ask_position(self):
        self.position_event.clear()
        self.position = None
        self.write_line('M114\n')
        
    def handle_line(self, data):
        if data[0] == '{':
            # We have a status!
            response = json.loads(data)
            if 'key' in response and response['key'] == "state.status":
                if response['result'] == 'idle':
                    self.idle.set()
                else:
                    self.ask_status()
        elif data == 'ok':
            pass
        elif data[0] == 'X':
            position = {}
            for x in data.split():
                if ':' not in x:
                    continue
                axis, location = x.split(':')
                position[axis] = float(location)
            self.position = position
            self.position_event.set()
        else:
            print("Unknown response", data)
            
    def connection_lost(self, exc):
        pass

class RRF3Driver(MachineDriver):

    
    def __init__(self):
        self.serial = serial.Serial('/dev/tty.usbmodem1421', baudrate=115200, timeout=1)
        self.connection = ReaderThread(self.serial,RRF3Connection)
        self.protocol = None
        
    def __enter__(self):
        self.protocol = self.connection.__enter__()
        self.protocol.ask_status()
        return self

    def __exit__(self, type, value, tb):
        self.connection.__exit__(type, value, tb)

    def move_to(self, position, velocity = None, block = False):
        command = f"""G0 X{position[0]} Y{position[1]} Z{position[2]}"""
        if velocity:
            command += f""" F{velocity}"""

        self.protocol.idle.wait()
        self.protocol.idle.clear()
        self.protocol.write_line(command + '\n')
        self.protocol.ask_status()
        
        if block:
            self.protocol.idle.wait()
        
    def finished(self):
        return self.protocol.idle.is_set()
        
    def wait(self):
        self.protocol.idle.wait()
    
    def get_current_position(self):
        self.protocol.ask_position()
        self.protocol.position_event.wait()
        pos = self.protocol.position
        return np.array([pos['X'], pos['Y'], pos['Z']])
    
    def home_z_axis(self):
        """ Reference the machine along the camera's optical axis - will not be called with a deployed camera. """
        pass

    def stow_camera(self):
        pass

    def deploy_camera(self):
        pass

#import numpy as np

#with RRF3Driver() as driver:
#
#    x,y,z = driver.get_current_position()#
#
#    for z in np.linspace(z, z + 1, 10):
#        driver.move_to((x,y,z), block = True)

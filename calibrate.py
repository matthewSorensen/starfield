import click



import time

import sys
import os
import pickle

import numpy as np
import time
from pewpew import MachineConnection, find_usbtty
from starfield.driver import MachineDriver, load_driver
from starfield.autofocus import autofocus
from starfield.mill_driver import get_planner, PewPewDriver
import cv2 as cv

import matplotlib
import matplotlib.pyplot as plt

from starfield.autofocus import autofocus

from starfield.optical_flow import OpticalFlowTracker
import cv2


def manual_focus_loop(camera, driver, increment = 0.1):
    cv.namedWindow("Camera Live Feed", cv.WINDOW_AUTOSIZE)
    while True:
        cv.imshow("Camera Live Feed", camera.read()[1])
        key = cv.waitKey(100)
        motion = None
        
        if key == ord(' ') or key == ord('q'):
            break
        elif key == ord('w'):
            motion = -1 * increment
        elif key == ord('s'):
            motion = increment
        elif key == ord('a'):
            increment *= 0.5
        elif key == ord('d'):
            increment *= 2

        if motion is not None:
            if driver.finished():
                x,y,z = driver.get_current_position()
                z += motion
                if z < 0:
                    continue
                driver.move_to((x,y,max(0.0,z)), block = False)
    driver.wait()
    cv.destroyAllWindows()

@click.command()
@click.option('--cv-camera', default = 0, type = int, help = "OpenCV VideoCapture ID for the desired camera - see OpenCV documentation for how to find this.")
@click.option('--resolution', default = "", type = str, help = "Desired camera capture resolution, in the format WIDTHxHEIGHT")

@click.option('--driver', default = "starfield.driver.MachineDriver", type = str, help = "Python path to a class that provides a machine driver implementation.")

@click.option('--focus-height', default = None, type = float, help = "Z height at which the calibration target is in focus. If provided, the autofocus pass is skipped.")
@click.option('--focus-coarse', default = None, type = float, help = "Initial guess for the focus plane. If not provided, a live camera view and jog interface will appear before the autofocus pass.")
@click.option('--focus-range', default = None, type = float, help = "How larger of a range should the autofocus pass search on either side of the focus height? If both this argument and --focus-coarse are provided, the autofocus pass will run without user input.")
@click.option('--no-home/--home', default = True, help = "Should the machine home/reference its z axis before starting the calibration process?")
@click.option('--target-thickness', default = 0, type = float, help = "How far is the target surface about the work surface of the machine? This parameter doesn't impact the calibration process, but is applied to saved focal plane offsets.")

@click.option('--tolerance', default = None, type = float, help = "How precise should we aim to be during the autofocus pass?")
@click.option('--discrete/--continuous', default = False, help = "Should we use a discrete, step-wise autofocus pass (slow, insensitive to controller interface) or a continuous (faster, more precise, requires better position feedback) one?")

@click.option('--field-size', default = None, type = float, help = "Initial guess for the length of the longest side of the camera's field of view.")

@click.argument('output', type=click.File('w'))

def calibrate(*args,**kwargs):

    with load_driver(kwargs['driver'])() as driver:
    
        # Get the camera and set its resolution, if that's provided
        camera = cv.VideoCapture(kwargs['cv_camera'])
        if not camera.isOpened():
            print("Invalid camera ID provided")
            exit(-1)
    
        if kwargs['resolution']:
            try:
                x,y = [int(x) for x in kwargs['resolution'].split('x')]
                camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, x)
                camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, y)
            except:
                print("Invalid camera resolution provided")
                camera.release()
                exit(-1)
        # Do we need to home the system?
        if kwargs['no_home']:
            driver.stow_camera()
            driver.home_z_axis()
            driver.deploy_camera()

        x,y,z = driver.get_current_position()
        # Get the initial guess for the focus height and search range
        coarse_focus = kwargs['focus_coarse']
        if coarse_focus is None:
            input("Jog the system until it is roughly in focus - press enter to begin ")
            manual_focus_loop(camera, driver)
            _,_,coarse_focus = driver.get_current_position()
        else:
            driver.move_to((x,y,coarse_focus))
            
        focus_range = kwargs['focus_range']
        if focus_range is None:
            input("Jog the system until it is out of focus - press enter to begin ")
            manual_focus_loop(camera, driver)
            _,_,focus_range = driver.get_current_position()
            focus_range = abs(coarse_focus - focus_range)

        # Autofocus with the supplied resolution and mode...
        # Then optical-flow track...
        z = autofocus(camera, driver, (coarse_focus - focus_range, coarse_focus + focus_range), 0.1, continuous = False)
        print(z)
        
        camera.release()

if __name__ == '__main__':
    callibrate()


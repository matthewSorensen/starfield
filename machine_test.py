import time

import sys
import os
import pickle

import numpy as np
from pewpew.planner import MotionPlanner, KinematicLimits
from pewpew import MachineConnection, find_usbtty, bind_halt_signal
from pewpew.definitions import StatusFlag, OverrideMessage

import time
from starfield.driver import MachineDriver
from starfield.mill_driver import get_planner, PewPewDriver
import cv2 as cv

import matplotlib
import matplotlib.pyplot as plt


def CMSL(img, window = 10):
    """
        Contrast Measure based on squared Laplacian according to
        'Robust Automatic Focus Algorithm for Low Contrast Images
        Using a New Contrast Measure'
        by Xu et Al. doi:10.3390/s110908281
        window: window size= window X window"""
    ky1 = np.array(([0.0, -1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]))
    ky2 = np.array(([0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, -1.0, 0.0]))
    kx1 = np.array(([0.0, 0.0, 0.0], [-1.0, 1.0, 0.0], [0.0, 0.0, 0.0]))
    kx2 = np.array(([0.0, 0.0, 0.0], [0.0, 1.0, -1.0], [0.0, 0.0, 0.0]))
    g_img = abs(cv.filter2D(img, cv.CV_32F, kx1)) + \
                abs(cv.filter2D(img, cv.CV_32F, ky1)) + \
                abs(cv.filter2D(img, cv.CV_32F, kx2)) + \
                abs(cv.filter2D(img, cv.CV_32F, ky2))
    return cv.boxFilter(g_img * g_img,-1,(window, window),normalize=True)


class Averager:
    def __init__(self,n):
        self.acc = 0
        self.i = 0
        self.n = n
        self.buffer = np.empty(n)
    def __call__(self,x):
        self.acc += x
        
        if self.i < self.n:
            self.buffer[self.i] = x
            self.i += 1
            return self.acc / self.i
        else:
            j = self.i % self.n
            self.acc -= self.buffer[j]
            self.buffer[j] = x
            self.i += 1
            return self.acc / self.n

def identity(x):
    return x
        
def continuous_sweep_autofocus(camera, driver, span, speed, roi = None, stop_early = None, flush = 5):
    position = driver.get_current_position()
    xy = position.copy()
    xy[2] = 0.0
    z = np.array([0,0,1.0])

    
    driver.move_to(xy + span[0] * z, block = True)

    focus = []
    position = []

    for _ in range(flush):
        _,_ = vid.read()
    
    driver.move_to(xy + span[1] * z, speed, block = False)
    while True:   
        ret, frame = vid.read()
        position.append(driver.get_current_position()[2])
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        focus.append(CMSL(gray,10).mean())
        if driver.finished():
            break

    return position, focus
    

def discrete_sweep_autofocus(camera, driver, span, zstep, roi = None, stop_early = False, flush = 2):

    position = driver.get_current_position()
    xy = position.copy()
    xy[2] = 0.0
    z = np.array([0,0,1.0])

    focus = []
    position = []
    first = True
    
    for zp in np.arange(span[0],span[1], zstep):
        driver.move_to(xy + zp * z, block = True)
        
        for _ in range(flush):
            _,_ = vid.read()
        
        ret, frame = vid.read()
        position.append(driver.get_current_position()[2])
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        focus.append(CMSL(gray,10).mean())
        
    return position, focus


focus = []
position = []

vid = cv.VideoCapture(0)
with MachineConnection(find_usbtty()) as m:
    
    driver = PewPewDriver(m, get_planner(m))


    #position, focus = continuous_sweep_autofocus(vid, driver, (-0.25,0.25), 2.0 / 60.0)
    #print(position)
    #print(focus)
    position, focus = discrete_sweep_autofocus(vid, driver, (-0.05,0.05), 0.001)
    print(position)
    print(focus)
    
    driver.move_to((0,0,0), block = True)


vid.release()
plt.plot(position, focus)
plt.show()



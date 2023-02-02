import time

import sys
import os
import pickle

import numpy as np
import time
from starfield.driver import MachineDriver
from starfield.mill_driver import get_planner, PewPewDriver
import cv2 as cv

import matplotlib
import matplotlib.pyplot as plt

from starfield.autofocus import focus_metric, continuous_sweep, discrete_sweep, Averager, MaximaTracker

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

focus = []
position = []

vid = cv.VideoCapture(0)
with MachineConnection(find_usbtty()) as m:
    
    driver = PewPewDriver(m, get_planner(m))

    metric = focus_metric()

    a = Averager(5)
    m = MaximaTracker()
    
    position, focus = continuous_sweep(vid, driver, (-0.25,0.25), 2.0 / 60.0, metric, stop_early = lambda x: m(a(x)))    
    driver.move_to((0,0,position[np.argmax(focus)]), block = True)


vid.release()
plt.plot(position, focus)
plt.show()



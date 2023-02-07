import numpy as np
import cv2 as cv
from enum import Enum
from dataclasses import dataclass
import time

def focus_metric(roi = None, average = True):

    """
        Contrast Measure based on squared Laplacian according to
        'Robust Automatic Focus Algorithm for Low Contrast Images
        Using a New Contrast Measure'
        by Xu et Al. doi:10.3390/s110908281 """
    ky1 = np.array(([0.0, -1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]))
    ky2 = np.array(([0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, -1.0, 0.0]))
    kx1 = np.array(([0.0, 0.0, 0.0], [-1.0, 1.0, 0.0], [0.0, 0.0, 0.0]))
    kx2 = np.array(([0.0, 0.0, 0.0], [0.0, 1.0, -1.0], [0.0, 0.0, 0.0]))
    
    def f(frame):
        if roi:
            frame = frame[roi[0]:roi[1],roi[2]:roi[3],:]
        img = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        filtered = np.abs(cv.filter2D(img,  cv.CV_32F, kx1))
        filtered += np.abs(cv.filter2D(img, cv.CV_32F, ky1))
        filtered +=  np.abs(cv.filter2D(img, cv.CV_32F, kx2))
        filtered +=  np.abs(cv.filter2D(img, cv.CV_32F, ky2))

        if average:
            return filtered.mean()
        return filtered
    
    return f

class Averager:
    """ Apply an n-value sliding window averaging to a stream of values. """
    def __init__(self,n):
        self.acc = 0
        self.i = 0
        self.n = n
        self.buffer = np.empty(n)
    def __call__(self,x):
        if self.n == 0:
            return x
        
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

class StoppingCriteria(Enum):
    DECREASE = 1
    PEAK = 2

class StoppingTracker:
    """ Detects two different early-stopping criteria for focus metrics:
    
    * A monotonic decrease for *continuous* samples, indicating we're moving away from the focus.
    
    * A monotonic increase for *continuous* samples, followed by a decrease under *threshold* of total
    peak height.
    
    """
    
    def __init__(self, continuous, threshold, average):
        self.average = Averager(average)
        # We need to track the extrema of the signal
        self.min, self.max, self.prev = None, None, None
        
        self.direction = None
        self.count = 0
        self.continuous = continuous
        self.threshold = threshold
        
    def __call__(self, x):
        x = self.average(x)
        if self.prev is None:
            self.prev = x
            self.min = x
            self.max = x
            return
        
        if x < self.min:
            self.min = x
        if x > self.max:
            self.max = x
        
        direction = self.prev < x
        if self.direction is None:
            self.direction = direction
            return
        
        if direction == self.direction:
            self.count += 1
            
            if self.count > self.continuous and self.continuous != -1:
                if not self.direction:
                    return StoppingCriteria.DECREASE
        else:
            self.count = 0
            self.direction = direction
        
        if self.count > self.continuous:
            # Once we've crossed that threshold once, always make sure we always take this branch.
            self.continuous = -1 
            
            if x < self.min + self.threshold * (self.max - self.min):
                return StoppingCriteria.PEAK
        
        self.prev = x


def continuous_sweep(camera, driver, span, speed, metric, stop_early = None, flush = 2):
    position = driver.get_current_position()
    xy = position.copy()
    xy[2] = 0.0
    z = np.array([0,0,1.0])

    if span[0] != position[2]:
        driver.move_to(xy + span[0] * z, block = True)

    focus = []
    position = []

    for _ in range(flush):
        _,_ = camera.read()

    can_stop = stop_early is not None and getattr(driver, 'halt', None) is not None
        
    driver.move_to(xy + span[1] * z, speed, block = False)
    while True:
        position.append(driver.get_current_position()[2])
        ret, frame = camera.read()
        m = metric(frame)
        focus.append(m)
        if driver.finished():
            break
        if can_stop:
            ret = stop_early(m)
            if ret is not None:
                driver.halt()
                return position, focus, ret

    return position, focus, None


def discrete_sweep(camera, driver, span, zstep, metric, stop_early = None, flush = 2):

    position = driver.get_current_position()
    xy = position.copy()
    xy[2] = 0.0
    z = np.array([0,0,1.0])

    focus = []
    position = []
    first = True
    if span[0] > span[1]:
        zstep *= -1
        
    for zp in np.arange(span[0],span[1], zstep):
        driver.move_to(xy + zp * z, block = True)
        
        for _ in range(flush):
            _,_ = camera.read()
        
        ret, frame = camera.read()
        m = metric(frame)
        position.append(zp)
        focus.append(m)

        if stop_early is not None:
            ret = stop_early(m)
            if ret is not None:
                return position, focus, ret
            
    return position, focus, None


@dataclass
class AutofocusOptions:
    # If a controller interface supports low-latency position feedback,
    # it is preferable to continuously sweep through the focus peak in one move, instead of
    # performing many small motions.
    continuous : bool = False

    # When evaluating early-stopping criteria and the final focal position, how many frames
    # should the focus metric be averaged over?
    average : int = 10

    # If the focus metric is seen to drop under this fraction of the maximum peak height,
    # relative to the starting value, the current sweep is considered finished.
    stopping_height : float = 0.5

    # How many frames must be discarded to always get the most recent camera frame?
    flush : int = 5

    # What is the current frame rate? If this is required and not present, it is determined
    # experimentally.
    framerate : float = None

    # Region of interest over which to evaluate the metric - (x low, x high, y low, y high)
    roi : object = None

    def get_framerate(self, camera, frames = 10):
        if self.framerate is not None:
            return self.framerate
        
        for _ in range(self.flush):
            camera.read()
            
        start = time.time()
        for _ in range(frames):
            camera.read()
        return frames / (time.time() - start)

    def merge(self,**kwargs):
        for k in self.__dict__.keys():
            if k in kwargs:
                self.__dict__[k] = kwargs[k]


def normalize_and_sort(distance, metric):
    distance,metric = np.array(distance), np.array(metric)
    idx = np.argsort(distance)
    
    return distance[idx], metric[idx] / max(metric)

def align_peaks(one, two, smooth):
    d1,f1 = normalize_and_sort(*one)
    d2,f2 = normalize_and_sort(*two)
    # Resample both onto a regular grid, and then smooth
    grid = np.linspace(min(d1[0],d2[0]),max(d1[-1],d2[-1]),2 * (len(d1) + len(d2)))
    window = np.ones(smooth)/smooth
    smooth1 = np.convolve(window,np.interp(grid, d1, f1),mode='valid')
    smooth2 = np.convolve(window,np.interp(grid, d2, f2),mode='valid')
    # Estimate the delay between the two signals, then compute the delay-adjusted average
    delay = 0.5 * (np.argmax(smooth1) - np.argmax(smooth2)) * (grid[1] - grid[0])
    summed = np.interp(grid, d1 - delay, f1) + np.interp(grid, d2 + delay, f2)
    return grid, summed, delay


def autofocus(camera, driver, span, resolution, options = None, **kwargs):
    if options is None:
        options = AutofocusOptions()
    options.merge(**kwargs)

    sweep = discrete_sweep
    if options.continuous: # How fast do we need to get to get our intended resolution?
        resolution = 0.5 * resolution * options.get_framerate(camera)
        sweep = continuous_sweep
        
    metric = focus_metric(options.roi)
    x,y,z = driver.get_current_position()
    lo, hi = min(span), max(span)
    near, far = (lo,hi) if abs(lo - z) < abs(hi - z) else (hi,lo)

    # Make the coarse sweep through the full range, significantly faster than the final speed, with early
    # stopping criteria.
    tracker = StoppingTracker(options.average, options.stopping_height, options.average)
    position, focus, reason = sweep(camera, driver, (near, far), 4 * resolution, metric, stop_early = tracker, flush = options.flush)

    if reason == StoppingCriteria.DECREASE:
        return None, "focus outside of interval"


    # Make a finer pass (or pair of passes) through a refined interval
    z_start = driver.get_current_position()[2]
    z_end =  position[np.argmax(np.array(focus) > focus[-1])]

    position, focus, reason = sweep(camera, driver, (z_start, z_end), resolution, metric, flush = options.flush)

    if options.continuous:
        
        p2,f2,_ = sweep(camera, driver, (z_end, z_start), resolution, metric, flush = options.flush)
        position,focus,_ = align_peaks((position,focus),(p2,f2),50)
        
    z = position[np.argmax(focus)]
    driver.move_to((x,y,z), block = True)

    return z

    

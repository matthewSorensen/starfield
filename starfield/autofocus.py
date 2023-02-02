import numpy as np
import cv2 as cv

def focus_metric(roi = None):

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

        return filtered.mean()

    return f

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

class MaximaTracker:
    
    def __init__(self, width = 0.5, increase_count = 10):
        self.worst = None
        self.best = None
        self.prev = None
        self.increases = 0
        self.increase_count = increase_count
        self.width = width
    def __call__(self, value):
        if self.best is None or self.best < value:
            self.best = value
        if self.worst is None or self.worst > value:
            self.worst = value
            
        if self.prev is None:
            self.prev = value
            return
        # Track how many successive increases we've seen?
        if value > self.prev:
            self.increases += 1
        else:
            self.increases = 0
            
        if self.increases > self.increase_count:
            self.increase_count = -1
            if value < self.worst + self.width * (self.best - self.worst):
                return True
    
        self.prev = value
        

def continuous_sweep(camera, driver, span, speed, metric, stop_early = None, flush = 2):
    position = driver.get_current_position()
    xy = position.copy()
    xy[2] = 0.0
    z = np.array([0,0,1.0])

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
        if can_stop and stop_early(m):
            driver.halt()
            break

    return position, focus


def discrete_sweep(camera, driver, span, zstep, metric, stop_early = None, flush = 2):

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
            _,_ = camera.read()
        
        ret, frame = camera.read()
        m = metric(frame)
        position.append(zp)
        focus.append(m)

        if stop_early is not None and stop_early(m):
            break
        
    return position, focus

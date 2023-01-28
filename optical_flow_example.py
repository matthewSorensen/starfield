import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from numpy.random import rand
import math
import cv2 as cv
import time

from starfield.optical_flow import OpticalFlowTracker

def gray_frames(file):
    cap = cv.VideoCapture(file)
    while True:
        ret, frame = cap.read()
        #print(ret)
        if not ret:
            break
        yield cv.cvtColor(frame, cv.COLOR_BGR2GRAY)


t0 = time.time()
frames = gray_frames("TestMovie.mp4")
for _ in frames:
    pass
print("Video loading time - ",time.time() - t0)


t0 = time.time()
frames = gray_frames("TestMovie.mp4")
tracker = OpticalFlowTracker()
for f in frames:
    tracker.process_frame(f)
    
tracker.finish()


print("Video loading + optical flow - ",time.time() - t0)

plt.imshow(tracker.prev_frame)

for i,p in tracker.tracks.items():
    pts = np.array(p)
    plt.plot(pts[:,0],pts[:,1])

plt.show()

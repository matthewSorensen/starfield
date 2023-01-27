import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from numpy.random import rand
import math
import cv2 as cv


def kabsch_umeyama(A, B):
    # Taken from: https://zpl.fi/aligning-point-patterns-with-kabsch-umeyama-algorithm/
    # Author released it as CC0 1.0 (Public Domain), so we're good!
    assert A.shape == B.shape
    n, m = A.shape

    EA = np.mean(A, axis=0)
    EB = np.mean(B, axis=0)
    VarA = np.mean(np.linalg.norm(A - EA, axis=1) ** 2)

    H = ((A - EA).T @ (B - EB)) / n
    U, D, VT = np.linalg.svd(H)
    d = np.sign(np.linalg.det(U) * np.linalg.det(VT))
    S = np.diag([1] * (m - 1) + [d])

    R = U @ S @ VT
    c = VarA / np.trace(np.diag(D) @ S)
    t = EA - c * R @ EB

    return R, c, t


n = 1000
feature_params = dict(maxCorners = n,qualityLevel = 0.3, minDistance = 7, blockSize = 7 )
lk_params = dict(winSize  = (15, 15), maxLevel = 3, criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

def gray_frames(file):
    cap = cv.VideoCapture(file)
    while True:
        ret, frame = cap.read()
        #print(ret)
        if not ret:
            break
        yield cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
frames = gray_frames("TestMovie.mp4")

# Identify the initial features from the first frame
prev_frame = next(frames)
initial_features = cv.goodFeaturesToTrack(prev_frame, mask = None, **feature_params)
idx = np.arange(initial_features.shape[0])
tracks = dict()
for i,x in enumerate(initial_features):
    tracks[i] = [x[0,:]]

def forward_track(initial_features, feature_numbers, prev_frame, frame_source, frames = 5, error_cutoff = 2):

    state = np.ones(initial_features.shape[0], dtype = 'uint8')
    updated_features = initial_features
    for new_frame in frame_source:
        updated_features, st, error = cv.calcOpticalFlowPyrLK(prev_frame, new_frame, updated_features, None, **lk_params)
        prev_frame = new_frame
        state *= st.T[0]
        frames -= 1
        if frames == 0:
            break
    
    _,_,t = kabsch_umeyama(initial_features[state==1,0,:],updated_features[state==1,0,:])
    error = np.linalg.norm(initial_features[:,0,:] - (t + updated_features[:,0,:]), axis = 1) < error_cutoff
    return prev_frame, updated_features[error], feature_numbers[error], t

motion = np.zeros(2)
features = initial_features
for i in range(80):
    prev_frame, features, idx, t = forward_track(features, idx, prev_frame, frames, frames = 10)
    motion += t
    for i,j in enumerate(idx):
        tracks[j].append(features[i,0,:])


for i,t in tracks.items():
    pts = np.array(t)
    plt.plot(pts[:,0],pts[:,1])

plt.show()

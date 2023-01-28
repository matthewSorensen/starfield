import cv2 as cv
import numpy as np

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


class OpticalFlowTracker:
    DefaultFeatureParams = dict(qualityLevel = 0.3,
                                minDistance = 7,
                                blockSize = 7)
    
    DefaultLKParams = dict(winSize  = (15, 15),
                           maxLevel = 3,
                           criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
    
    DefaultParams = dict(max_features = 1000,
                           frame_batch = 10,
                           full_trace = False,
                           max_tracking_error = 2,
                           feature_params = None,
                           lk_params = None)

    def __init__(self,**kwargs):
        OpticalFlowTracker.DefaultParams['feature_params'] = OpticalFlowTracker.DefaultFeatureParams
        OpticalFlowTracker.DefaultParams['lk_params'] = OpticalFlowTracker.DefaultLKParams
        # Merge any user-overidden parameters, including in the sub-parameter dictionaries.
        parameters = {**OpticalFlowTracker.DefaultParams,**kwargs}
        for key in ['feature_params','lk_params']:
            if key not in kwargs:
                continue
            parameters[key] = {**OpticalFlowTracker.DefaultParams[key],**kwargs[key]}
        parameters['feature_params']['maxCorners'] = parameters['max_features']
        self.parameters = parameters
        self.tracks = {}
        self.features = None
        self.raw_features = None
        self.state = None
        self.prev_frame = None
        self.feature_idx = None
        self.i = 0
        
    def process_frame(self, frame):
        if self.prev_frame is None:
            self.prev_frame = frame
            self.features = cv.goodFeaturesToTrack(frame, mask = None, **self.parameters['feature_params'])
            self.feature_idx = np.arange(self.features.shape[0])
            full = self.parameters['full_trace']
            for i,x in enumerate(self.features):
                if full:
                    self.tracks[i] = [x[0,:]]
                else:
                    self.tracks[i] = [x[0,:],x[0,:]]

            self.raw_features = self.features
            self.state = np.ones(self.features.shape[0], dtype = 'uint8')
            return
        # Compute the flow for this frame
        self.raw_features, state, _ = cv.calcOpticalFlowPyrLK(self.prev_frame, frame, self.raw_features, None, **self.parameters['lk_params'])
        self.state *= state.T[0]
        self.prev_frame = frame
        # Check if this was the last frame in the batch, and if
        # so run the backwards check.
        self.i += 1
        if self.i == self.parameters['frame_batch']:
            self.finish()
            self.i = 0
        
    def finish(self):
        # Find the best ridid-body transform between the features that survived this round of flow
        # and the previous checkpoint.
        _,_,t = kabsch_umeyama(self.features[self.state==1,0,:],self.raw_features[self.state==1,0,:])
        # Apply the inverse transformation to the new points, compare that to the old points,
        # and reject all feature points with too much error
        error = np.linalg.norm(self.features[:,0,:] - (t + self.raw_features[:,0,:]), axis = 1)
        good = error < self.parameters['max_tracking_error']
        # Record the traces of each point - if died, remove from the traces, otherwise note the current
        # position.
        full = self.parameters['full_trace']
        for i,g in enumerate(good):
            j,x = self.feature_idx[i], self.features[i,0,:]
            if not g:
                del self.tracks[j]
            elif full:
                self.tracks[j].append(x)
            else:
                self.tracks[j][-1] = x
        # Reject all the bad points and shrink the current state
        self.features = self.raw_features[good]
        self.raw_features = self.features    
        self.feature_idx = self.feature_idx[good]
        self.state = np.ones(self.features.shape[0], dtype = 'uint8')

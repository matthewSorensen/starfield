import numpy as np

@dataclass
class CameraModel:
    # 5x2 transform matrix, mapping a quadratic basis ([x,y,x*y...])
    # of normalized pixel coordinates to real-world offsets from the center of the frame.
    transform : object
    # The size of the frame
    size : (int,int)
    # An estimate of the worst case residual (in real-space units) error introduced by
    # the model.
    uncertainty : float
    # How big is a pixel in real-space units? Due to non-unform scaling, this is a range of
    # points.
    pixel_scale : (float,float)
    # Rotation angle between camera coordinates and machine coordinates
    rotation : float
    # Reference focal plane - this is not directly used, but is convenient to pass
    # around with calibrations. The cannonical interpretation is the z value when the 
    # camera is focused on machine work surface - the plane where all object heights are
    # calculated from.
    focal_plane : float
    
    @staticmethod
    def estimate_from_deltas(obs, size, focus):
        """ Takes a list of observed line segments from optical flow, with each element in the form
    (start x, start y, end x, end y, true x, true y), and the frame size, and generates a camera
        calibration from it. """
        # Pack and normalize the pixel coordinates to [-0.5,0.5]
        obs = np.array(observations, dtype = float)
        obs[:,0] /= size[0]
        obs[:,1] /= size[1]
        obs[:,2] /= size[0]
        obs[:,3] /= size[1]
        obs[:,0:4] -= 0.5
        # Set up and solve linear system - we're using B(x,y) = [x,y,x*y,x^2,y^2] as a basis
        # for our model, so we want to solve H(B(end) - B(start)) = true for the distortion
        # matrix H
        m = np.vstack([obs[:,2] - obs[:,0],
                       obs[:,3] - obs[:,1],
                       obs[:,2] * obs[:,3] - obs[:,0] *obs[:,1],
                       obs[:,2]**2 - obs[:,0]**2,
                       obs[:,3]**2 - obs[:,1]**2]).T
        H,_,_,_ = np.linalg.lstsq(m,obs[:,4:], rcond = None)
        return CameraModel.build_from_estimate(H, m @ H - obs[:,4:], size, focus)

    @staticmethod
    def build_from_estimate(transform, residual, size, focus):
        _,scale,rotate = np.linalg.svd(transform[0:2,0:2] @ np.diag([1 / size[0], 1 / size[0]]))
        uncertainty = np.max(np.linalg.norm(residual, axis = 1))
        return CameraModel(transform, size, uncertainty, (min(scale),max(scale)), math.acos(rotate[0,0]), focus)

    # Serialize/deserialize from json
    def serialize(self, filename = None):
        d = deepcopy(self.__dict__)
        d['transform'] = d['transform'].tolist()
        if filename is not None:
            with open(filename,'w') as f:
                json.dump(d,f)
        return d
    @staticmethod
    def deserialize(filename = None, dictionary = None):
        if filename is not None:
            with open(filename,'r') as f:
                dictionary = json.loads(f.read())
        if dictionary is None:
            return None
        dictionary['transform'] = np.array(dictionary['transform'], dtype = float)
        return CameraModel(**dictionary)
    
    def apply(self,points):
        points = np.array(points)
        original_shape = points.shape
        points = points.reshape((-1,2))
        x = points[:,0] / self.size[0] - 0.5
        y = points[:,1] / self.size[1] - 0.5      
        return (self.transform.T @ np.vstack([x,y,x*y,x**2,y**2])).reshape(original_shape)
    
    def inverse_transform(self, points, max_iterations = 10):
        points = np.array(points)
        original_shape = points.shape
        points = points.reshape((-1,2))
        # Take the linear part of the transfer, invert it, and make
        # a first guess by applying that to the points
        inv = np.linalg.inv(self.transform[0:2,0:2])
        guess = inv @ points.T
        
        for _ in range(max_iterations):
    
            x,y = guess
            error = (transform.T @ np.vstack([[x,y,x*y,x**2,y**2]])).T - points
            guess -= inv @ error.T

            if max(np.abs(error)) < self.pixel_scale[1]:
                break
            
            
        return guess.reshape(original_shape)

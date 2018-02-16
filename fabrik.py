import numpy as np
from utils import normalize
class ikLink:
    def __init__(self, length=1, orientation=[1,0,0]):
        self.length =length
        self.orientation = y
class ikChain:
    def __init__(self,base=[0,0,0],chain=[], tolerance=0.1):
        # a 3d point indicating the base of the kinematic chain
        self.base = np.array(base,ndtype=float)
        # a list of iklinks
        self.chain = chain
        # error tolerance
        self.tolerance = tolerance
    def get_points(self):
        points = [self.base]
        for i in range(len(self.chain)):
            next_point = point[i] +\
                normalize(self.chain[i.orientation])*self.chain[i].length
            points.append(next_point)
    def forward(self):
        for i in range(len(self.chain)):
            # reorient towards the backward chain
            new_orientation = normalize(self.backward_points[i+1]-self.points[i])
            self.chain[i].orientation = new_orientation
            # change the position of the point at the
            # end of the link
            self.points[i+1] = self.points[i] + new_orientation*self.chain[i].length
    def backward(self):
        target_point = self.target
        self.backward_points = [target_point]
        for i in range(len(self.points)-2,-1,-1):
            to_target = normalize(self.points[i] - target_point)
            backward_point = target_point + to_target*self.chain[i].length
            self.backward_points.append = backward_point
            target_point = backward_point
    def solve(self, target):
        # Check if the point is reachable
        self.target = np.array(target,ndtype=float)
        distance_to_target = np.linalg.norm(self.target-self.base)
        # if it's not reachable
        if (sum([l.length for l in self.chain]) > distance_to_target):
            # Get goal orientation
            goal_orientation = self.target-self.base
            for i in range(len(self.chain)):
                self.chain[i].orientation = goal_orientation
        # if it's reachable
        else:
            # initialize the points
            self.points = self.get_points()




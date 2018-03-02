import numpy as np
from utils import normalize, distance
from vpython import *

class ikLink:
    def __init__(self, length=1, orientation=[1,0,0]):
        self.length = length
        self.orientation = y

class ikChain:
    def __init__(self,base=[0,0,0],chain=[], tolerance=0.1, iterations=10):
        # a 3d point indicating the base of the kinematic chain
        self.base = np.array(base,dtype=float)
        # a list of iklinks
        self.chain = chain
        # error tolerance
        self.tolerance = tolerance
        self.iterations = iterations

    def init_skeleton(self):
        # initialize the points
        self.points = self.get_points()
        # draw a box on the base
        self.base_box = box(pos=vec(*self.base), length=10, height=3, width=10)
        #draw each element in the ik chain
        self.graphic_ik = []
        for index in range(len(self.chain)):
            # Normalize the orientation o f the ik link
            pos = vec(*self.point[index])
            axis = vec(*(self.chain[index].orientation*self.chain[orientation].length))
            self.chain[index].orientation = normalize(self.chain[index].orientation)
            joint =  sphere(pos=pos,color=color.green, radius = 4)
            link = cylinder(pos=pos, axis=axis, radius=2)
            self.chain.append(joint)
            self.chain.append(link)

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
            # get distance between target and goal 
            error = distance(self.points[-1],self.target)
            count = 0
            while error > self.tolerance:
                self.backward()
                self.forward()
                error = distance(self.points[-1],self.target)
                if count > self.iterations:
                        break
                count += 1
    def animate(self):
      rate(100)
      # self.t = self.t + self.dt
      # self.ball.pos = self.ball.pos + (self.ball.p/self.ball.mass)*self.dt
      # if not (self.side > self.ball.pos.x > -self.side):
        # self.ball.p.x = -self.ball.p.x
      # if not (self.side > self.ball.pos.y > -self.side):
        # self.ball.p.y = -self.ball.p.y
      # if not (self.side > self.ball.pos.z > -self.side):
        # self.ball.p.z = -self.ball.p.z

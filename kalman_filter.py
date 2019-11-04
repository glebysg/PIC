 # Kalman filter example demo in Python

# A Python implementation of the example given in pages 11-15 of "An
# Introduction to the Kalman Filter" by Greg Welch and Gary Bishop,
# University of North Carolina at Chapel Hill, Department of Computer
# Science, TR 95-041,
# https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf

# by Andrew D. Straw

import numpy as np
import matplotlib.pyplot as plt
from helpers import kalman_filter

plt.rcParams['figure.figsize'] = (10, 8)
# Define arm joints
left_h_joints = [4,5,6]
right_h_joints = [8,9,10]
# intial parameters
# x = -0.37727 # truth value (typo in example at top of p. 13 calls this z)
task = 'incision_curvy'
data_version = '1'
skel_path = './data/new/'+task+data_version+'_skel.txt'
skel_reader = open(skel_path, 'r')
data_point = np.array(skel_reader.readline().split(), dtype=float)

human_l = []
human_r = []
# Get the data
n_iter = 0
for line in skel_reader:
    point_l = []
    point_r = []
    for l_index, r_index in zip(left_h_joints, right_h_joints):
        point_l.append([-data_point[l_index*3],
                data_point[l_index*3+1],
                -data_point[l_index*3+2]])
        point_r.append([-data_point[r_index*3],
                data_point[r_index*3+1],
                -data_point[r_index*3+2]])
    human_l.append(point_l)
    human_r.append(point_r)
    n_iter += 1
sz = (n_iter,) # size of array
skel_reader.close()
human_l = np.array(human_l)
human_r = np.array(human_r)
print(human_l.shape)
print(human_r.shape)

z = human_l[:,2,2]
# z = np.random.normal(x,0.1,size=sz) # observations (normal about x, sigma=0.1)
Q = 1e-5 # process variance

xhat, Pminus = kalman_filter(z, Q)

plt.figure()
plt.plot(z,'k+',label='noisy measurements')
plt.plot(xhat,'b-',label='a posteri estimate')
# plt.axhline(x,color='g',label='truth value')
plt.legend()
plt.title('Estimate vs. iteration step', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('Voltage')

plt.figure()
valid_iter = range(1,n_iter) # Pminus not valid at step 0
plt.plot(valid_iter,Pminus[valid_iter],label='a priori error estimate')
plt.title('Estimated $\it{\mathbf{a \ priori}}$ error vs. iteration step', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('$(Voltage)^2$')
plt.setp(plt.gca(),'ylim',[0,.01])
plt.show()

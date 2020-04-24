from sympy import *
from helpers import *
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as plt3d

# create the symbolic variables that will represent
# the joint angles
left_joint_names = ("lt1, lt2, lt3, lt4, lt5, lt6, lt7")
right_joint_names = ("rt1, rt2, rt3, rt4, rt5, rt6, rt7")
left_joint_vars = symbols(left_joint_names)
print(left_joint_vars)
right_joint_vars = symbols(right_joint_names)
left_neutral = (0,-31,0,43,0,72,0)
# Define the DH parameter matrix for the Baxter
# following the format:
# alpha(degrees), a(mm), d(mm), theta(degrees),
# range from(degrees), range to(degrees)
dh_left = Matrix([
    [  0,  0,      0, left_joint_vars[0]],
    [-90, 69,      0, left_joint_vars[1] + 90],
    [ 90,  0, 364.35, left_joint_vars[2]],
    [-90, 69,      0, left_joint_vars[3]],
    [ 90,  0, 374.29, left_joint_vars[4]],
    [-90, 10,      0, left_joint_vars[5]],
    [ 90,  0,      0, left_joint_vars[6]],
    ])

#get_robot_points
t_list = get_transformations(dh_left, left_joint_vars)
arm_points = [[0,0,0]]
accum_t = np.identity(4)
for i in range(len(t_list)):
    t_i = t_list[i]
    accum_t = np.dot(accum_t, t_i(*left_neutral))
    # get the translation (sice we do not care about the rotation of a point)
    arm_points.append(accum_t[0:3,3])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
print(arm_points)

# draw the lines
for i in range(1,len(arm_points)):
    line_x = [arm_points[i-1][0], arm_points[i][0]]
    line_y = [arm_points[i-1][1], arm_points[i][1]]
    line_z = [arm_points[i-1][2], arm_points[i][2]]
    line = plt3d.art3d.Line3D(line_x, line_y, line_z)
    ax.add_line(line)
plt.show()



from sympy import *
from helpers import *
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as plt3d
from vpython import *

# create the symbolic variables that will represent
# the joint angles
left_joint_names = ("lt1, lt2, lt3, lt4, lt5, lt6, lt7")
right_joint_names = ("rt1, rt2, rt3, rt4, rt5, rt6, rt7")
left_joint_vars = symbols(left_joint_names)
right_joint_vars = symbols(right_joint_names)
neutral = (0,-31,0,43,0,72,0)
home=(0,0,0,0,0,0,0)
pose = neutral

# Define the DH parameter matrix for the Baxter
# following the format:
# alpha(degrees), a(mm), d(mm), theta(degrees),
# range from(degrees), range to(degrees)
torso_l= Matrix([
    [0.707, -0.707, 0, 0.0638],
    [0.707,  0.707, 0, 0.259],
    [0, 0, 1, 0.1763],
    [0, 0, 0, 1]
])

torso_r= Matrix([
    [ 0.707,  0.707, 0, 0.0642],
    [-0.707,  0.707, 0, -0.259],
    [0, 0, 1, 0.1763],
    [0, 0, 0, 1]
])

dh_left = Matrix([
    [-90 ,0.0690 ,0.2703 ,left_joint_vars[0]],
    [90 ,0 ,0 ,left_joint_vars[1]+90],
    [-90 ,0.0690 ,0.3644 ,left_joint_vars[2]],
    [90 ,0 ,0 ,left_joint_vars[3]],
    [-90 ,0.01 ,0.3743 ,left_joint_vars[4]],
    [90 ,0 ,0 ,left_joint_vars[5]]
])

dh_right = Matrix([
    [-90 ,0.0690 ,0.2703 ,right_joint_vars[0]],
    [90 ,0 ,0 ,right_joint_vars[1]+90],
    [-90 ,0.0690 ,0.3644 ,right_joint_vars[2]],
    [90 ,0 ,0 ,right_joint_vars[3]],
    [-90 ,0.01 ,0.3743 ,right_joint_vars[4]],
    [90.0 ,0 ,0 ,right_joint_vars[5]]
])

# gripper = Matrix([
    # [1, 0, 0, 0],
    # [0, 1, 0, 0],
    # [0, 0, 1, 0.3683],
    # [0, 0, 0, 1]])

gripper = Matrix([
    [0, -1, 0, 0],
    [1, 0, 0, 0],
    [0, 0, 1, 0.1134],
    [0, 0, 0, 1]])

# joint_range=[
# -9.750e+01, 1.950e+02
# 1.950e+02, 1.950e+02
# -1.750e+02, 3.500e+02
# -2.865e+00, 1.529e+02
# -1.753e+02, 3.505e+02
# -9.000e+01, 2.100e+02
# 2.100e+02,3.505e+02
# ],


#get_robot_points
# Start with the world transformation
l_t_list = [lambdify(left_joint_vars,torso_l,'numpy')]
r_t_list = [lambdify(right_joint_vars,torso_r,'numpy')]
# add dh transformations
l_t_list += get_transformations(dh_left, left_joint_vars)
r_t_list += get_transformations(dh_right, right_joint_vars)
# l_t_list = get_transformations(dh_left, left_joint_vars)
# r_t_list = get_transformations(dh_right, right_joint_vars)
# add gripper
l_t_list.append(lambdify(left_joint_vars,gripper,'numpy'))
r_t_list.append(lambdify(right_joint_vars,gripper,'numpy'))

l_arm_points = [[0,0,0]]
r_arm_points = [[0,0,0]]
l_accum_t = np.identity(4)
r_accum_t = np.identity(4)
for i in range(len(l_t_list)):
    l_t_i = l_t_list[i]
    r_t_i = r_t_list[i]
    l_accum_t = np.dot(l_accum_t, l_t_i(*pose))
    r_accum_t = np.dot(r_accum_t, r_t_i(*pose))
    # draw the frames from the perspective of the coopelia frame
    draw_reference_frame(r_accum_t, transform=True)
    draw_reference_frame(l_accum_t, transform=True)
    # get the translation (sice we do not care about the rotation of a point)
    l_arm_points.append(l_accum_t[0:3,3])
    r_arm_points.append(r_accum_t[0:3,3])
l_arm_points = np.array(l_arm_points)
r_arm_points = np.array(r_arm_points)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
print(l_arm_points)
print(l_t_list[0](*left_joint_vars))

# ax.scatter(l_arm_points[1:,0], l_arm_points[1:,1], l_arm_points[1:,2], marker='^')
# ax.scatter(r_arm_points[1:,0], r_arm_points[1:,1], r_arm_points[1:,2], marker='^')

# LEFT POSITIONS
left_pos = np.array([
[0.063803821802139, 0.25905722379684, 0.17630106210709],
[0.112593986094   , 0.30784836411476, 0.44664913415909],
[0.18471896648407 , 0.37997275590897, 0.44660693407059],
[0.37026315927505 , 0.56551533937454, 0.37752240896225],
[0.44351378083229 , 0.63876533508301, 0.37747138738632],
[0.63492566347122 , 0.83017534017563, 0.36735194921494],
[0.71693193912506 , 0.91218090057373, 0.36728006601334]])
# y_l = -left_pos[:,1]
# left_pos[:,1]=left_pos[:,2]
# left_pos[:,2]=y_l
# RIGHT POSITIONS
right_pos = np.array([
[0.064155280590057, -0.25897043943405, 0.17630106210709],
[0.11295724660158 , -0.30777248740196, 0.44664055109024],
[0.18507945537567 , -0.37989434599876, 0.44661432504654],
[0.37062647938728 , -0.56544142961502, 0.37754958868027],
[0.44387751817703 , -0.63869214057922, 0.37752658128738],
[0.63528883457184 , -0.83010226488113, 0.36741954088211],
[0.71729582548141 , -0.91210895776749, 0.36738532781601]])


# y_r = -right_pos[:,1]
# right_pos[:,1]=right_pos[:,2]
# right_pos[:,2]=y_r
# left_pos = left_pos*100
# right_pos = right_pos*100
# draw the lines
# for i in range(0,len(l_arm_points)):
    # # Left
    # l_line_x = [l_arm_points[i-1][0], l_arm_points[i][0]]
    # l_line_y = [l_arm_points[i-1][1], l_arm_points[i][1]]
    # l_line_z = [l_arm_points[i-1][2], l_arm_points[i][2]]
    # l_line = plt3d.art3d.Line3D(l_line_x, l_line_y, l_line_z)
    # ax.add_line(l_line)
    # # right
    # r_line_x = [r_arm_points[i-1][0], r_arm_points[i][0]]
    # r_line_y = [r_arm_points[i-1][1], r_arm_points[i][1]]
    # r_line_z = [r_arm_points[i-1][2], r_arm_points[i][2]]
    # r_line = plt3d.art3d.Line3D(r_line_x, r_line_y, r_line_z)
    # ax.add_line(r_line)
    # ax.auto_scale_xyz
# plt.show()

# y_l = -l_arm_points[:,1]
# y_r = -r_arm_points[:,1]
# r_arm_points[:,1]=r_arm_points[:,2]
# l_arm_points[:,1]=l_arm_points[:,2]
# r_arm_points[:,2]=y_r
# l_arm_points[:,2]=y_l
coppelia_left_arm = []
coppelia_right_arm = []
row,col=l_arm_points.shape
for i in range(row):
    h_left = np.append(l_arm_points[i,:],1)
    h_right = np.append(r_arm_points[i,:],1)
    print("h_left", h_left)
    coppelia_left_arm.append(coppelia_pt_to_vpython(h_left))
    coppelia_right_arm.append(coppelia_pt_to_vpython(h_right))
coppelia_left_arm = np.array(coppelia_left_arm)
coppelia_right_arm = np.array(coppelia_right_arm)

coppelia_left_points = []
coppelia_right_points = []
row,col=left_pos.shape
for i in range(row):
    h_left = np.append(left_pos[i,:],1)
    h_right = np.append(right_pos[i,:],1)
    coppelia_left_points.append(coppelia_pt_to_vpython(h_left))
    coppelia_right_points.append(coppelia_pt_to_vpython(h_right))
coppelia_left_points = np.array(coppelia_left_points)
coppelia_right_points = np.array(coppelia_right_points)

draw_coppelia_reference_frame(0,0,0,arrow_size=10)
# draw_debug(coppelia_left_points*100,color.blue)
# draw_debug(coppelia_right_points*100,color.blue)
draw_debug(coppelia_left_arm[1:]*100,color.yellow)
draw_debug(coppelia_right_arm[1:]*100,color.orange)

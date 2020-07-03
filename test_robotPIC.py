from sympy import *
from helpers import *
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as plt3d
from vpython import *
from copy import copy
from pic import robotLink, robotChain
from copy import deepcopy
from datetime import timedelta
from PIL import Image
from time import sleep
from scipy import integrate
import os
import argparse
import sys
from os import path
import json
########## PARAMS ##############
# IMPORTANT: the distance units are in
# centimeters for rendering purposes

# example call:
# python incision_task.py -s 3 -t incision_curvy -v 1 -r Baxter -o ./data/results/incision_straight.txt

##############################
###     PARSE ARGS         ###
##############################
iterations = 20
parser = argparse.ArgumentParser()
parser.add_argument('-s', action="store", dest="soften", default=3,
        type=int, help="degree of softening for the imitation algorithm,\
                the valiues can go from 1 to 3")
parser.add_argument('-w', action="store", dest="sleep_time", default=0.02,
        type=float, help="waiting time between datapoint updates ")
parser.add_argument('-p', action="store", dest="pose_threshold", default=0.2,
        type=float, help="Percentage of the angle range that will be use to determine if the pose is acccurate")
parser.add_argument('-t', action="store", dest="task", default='incision_straight',
        help="name of the task to execute, example 'incision_straight'")
parser.add_argument('-v', action="store", dest="data_version",
        default="3", help="version of the task that is executed (a number)")
parser.add_argument('-r', action="store", dest="robot",
        default="baxter", help="Robot to use in the task. The options are Baxter or Yumi")
parser.add_argument('-d', action="store", dest="data_path", default="./data/new/",
        help="Path location of the skel files")
parser.add_argument('-c', action="store", dest="config_path", default="./simulation/arms/",
        help="Path location of the config files")
parser.add_argument('-a', action="store", dest="annotation_path", default="./simulation/data_points/",
        help="Path location of the annotated datapoint files")
parser.add_argument('-o', action="store", dest="output_path", default="./data/results/",
        help="Path location of the output result files")
parser.add_argument('-f', action="store", dest="filter", default="exp",
        help="type of filterered data to use. Options: exp, kalman, double")
parser.add_argument('--pose_imitation',action="store_true", default=False,
        help="if present, it uses the pose imitation Algorithm")
parser.add_argument('--conic_constraints',action="store_true", default=False,
        help="if present, it uses the pose imitation with conic constraints")
parser.add_argument('--append',action="store_true", default=False,
        help="if present, append to the file instead of rewriting it")
parser.add_argument('--filtering',action="store_true", default=False,
        help="if present, it smoothens the movements through filtering")
parser.add_argument('--threshold', action="store", dest="filtering_threshold", default=10,
        type=int, help="threshold of change in degrees \
                at whitch the robot movement is fitered")
args = parser.parse_args()
soften = args.soften
sleep_time = args.sleep_time
pose_threshold = args.pose_threshold
robot = args.robot
data_version = args.data_version
task = args.task
pose_imitation = args.pose_imitation
filtering = args.filtering
filter_threshold = args.filtering_threshold
file_append = args.append
smoothing = "smooth_"
if args.filter == 'kalman' or args.filter == 'double':
    smoothing = args.filter + "_" + smoothing
skel_path = args.data_path+smoothing+task+data_version+'_skel.txt'
ts_path = args.data_path+task+'_skelts.txt'
task_path = args.annotation_path+task+'_'+data_version+'.txt'
pad_path  = "./simulation/textures/pad.jpg"
arm_path = args.config_path+robot+'.txt'
robot_config_path = args.config_path+robot+'_config_'\
        +task+data_version+'.json'
print(task_path)
print(robot_config_path)
task_datapoints = np.loadtxt(task_path, delimiter=' ')
out_file = path.join(args.output_path,task+data_version+"_"+robot+"_")

###############################
# read robot config
config_reader = open(robot_config_path)
robot_config = json.load(config_reader)
base = robot_config["base"]
human_joint_index = robot_config["human_joint"]
init_constraints = robot_config["constraints"]
# Use conic constraints only when presnet in the config an indicated
# through the arguments
if args.conic_constraints and "conic_constraints" in robot_config:
    conic_constraints = robot_config["conic_constraints"]
    print("CONIC CONSTRAINTS, ", conic_constraints)
else:
    conic_constraints = None
scale = robot_config["scale"]
task_arm = robot_config["arm"]
pad_offset = vec(*robot_config["pad_offset"])
pad_axis = vec(*robot_config["pad_axis"])
pad_dim = robot_config["pad_dim"]

# get the algorigthm name for the robot file
if args.conic_constraints and "conic_constraints" in robot_config:
    algorithm = 'posecones'
elif pose_imitation:
    algorithm = 'poseimit'
    algorithm += str(soften)
else:
    algorithm = 'fabrik'
out_file += algorithm+".txt"

################## scene setup ################################
scene = canvas(title='Pose imitation experiments', width=1200, height=800)
draw_vpython_reference_frame(0,0,0,arrow_size=10)

################### incision/assembly pad  ######################
pad_points = []
length = pad_dim[0]*scale
height = pad_dim[1]*scale
width = pad_dim[2]*scale
if task == "assembly":
    # occlussion calculation pad
    pad = box(pos=pad_offset, length=length, height=height, width=width,
            axis=pad_axis,  opacity=0.5, color=color.white)
    pad.visible = False
    # create the two pieces of assembly
    # width=width, texture={'file': pad_path, place:['right']})
else:
    # surgical pad
    pad = box(pos=pad_offset, length=length, height=height,
            width=width, texture=pad_path)

####### Get pad plane ##########
# if the occlussion/pad is in the x-z plane
if np.argmax(pad_axis.value) == 0:
    pad_points.append((pad.pos+vec(-length, height, -width)/2).value)
    pad_points.append((pad.pos+vec(-length, height, width)/2).value)
    pad_points.append((pad.pos+vec(length, height, width)/2).value)
    pad_points = np.array(pad_points)
    pad_normal = get_plane_normal(pad_points)
    # get the dimentions of the pad that are alinged with the robot's
    # corrdinate system
    pad_proj = length
    pad_orth = width
    # get the coordinate systems indexes that match the pads x,y
    # coordinate system
    pad_x_index = 0
    pad_y_index = 2
# if the occlussion/pad is in the z-y plane
elif np.argmax(pad_axis.value) == np.argmin(pad_dim):
    pad_points.append((pad.pos+vec(height, -length, -width)/2).value)
    pad_points.append((pad.pos+vec(height, length, -width)/2).value)
    pad_points.append((pad.pos+vec(height, length, width)/2).value)
    pad_points = np.array(pad_points)
    pad_normal = get_plane_normal(pad_points)
    # get the dimentions of the pad that are alinged with the robot's
    # corrdinate system
    pad_proj = length
    pad_orth = width
    # get the coordinate systems indexes that match the pads x,y
    # coordinate system
    pad_x_index = 1
    pad_y_index = 2
# if the occlussion/pad is in the x-y plane
else:
    pad_points.append((pad.pos+vec(-height, -length, width)/2).value)
    pad_points.append((pad.pos+vec(-height, length, width)/2).value)
    pad_points.append((pad.pos+vec(height, length, width)/2).value)
    pad_points = np.array(pad_points)
    pad_normal = get_plane_normal(pad_points)
    # get the dimentions of the pad that are alinged with the robot's
    # corrdinate system
    pad_proj = height
    pad_orth = length
    # get the coordinate systems indexes that match the pads x,y
    # coordinate system
    pad_x_index = 0
    pad_y_index = 1
print(pad_points[0])

########### Create the robot #######################

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
    [90 ,0 ,0 ,left_joint_vars[5]],
    [90.0, 0, 0.3324, left_joint_vars[6]]
])

dh_right = Matrix([
    [-90 ,0.0690 ,0.2703 ,right_joint_vars[0]],
    [90 ,0 ,0 ,right_joint_vars[1]+90],
    [-90 ,0.0690 ,0.3644 ,right_joint_vars[2]],
    [90 ,0 ,0 ,right_joint_vars[3]],
    [-90 ,0.01 ,0.3743 ,right_joint_vars[4]],
    [90.0 ,0 ,0 ,right_joint_vars[5]],
    [90.0, 0, 0.3324, right_joint_vars[6]]
])

ignore_tr = [1,0,0,0,0,0,0,0]
# if the leght of "d" should come before the legth "a"
# when rendering the robots
d_first = [1,1,1,1,1,1,1]

joint_range=np.array([
    [-9.750e+01, 1.950e+02],
    [ 1.950e+02, 1.950e+02],
    [-1.750e+02, 3.500e+02],
    [-2.865e+00, 1.529e+02],
    [-1.753e+02, 3.505e+02],
    [-9.000e+01, 2.100e+02],
    [ 2.100e+02, 3.505e+02],
])

# TODO add this info to the new config files
human_joint_index = [0,3,6]
# Define arm joints
left_h_joints = [4,5,6]
right_h_joints = [8,9,10]

#### Initialize the robot ######
arm_l = robotChain(dh_params=dh_left,joint_vars=left_joint_vars,ranges=joint_range,
        base_matrix=torso_l, pose_imitation=pose_imitation,
        human_joint_index=human_joint_index,
        iterations=iterations, soften=soften, filtering=filtering,
        filter_threshold=filter_threshold,
        render_task=task, render_scale=scale)
arm_l.init_skeleton(home, d_first)
arm_r = robotChain(dh_params=dh_right,joint_vars=right_joint_vars,ranges=joint_range,
        base_matrix=torso_r, pose_imitation=pose_imitation,
        human_joint_index=human_joint_index,
        iterations=iterations, soften=soften, filtering=filtering,
        filter_threshold=filter_threshold,
        render_task=task, render_scale=scale)
arm_r.init_skeleton(home, d_first)

#######################################################
direction = None
human_l_chain = []
human_r_chain = []
skel_reader = open(skel_path, 'r')
line_counter = 0
rate(30)
task_metrics = []

for current_point, current_arm in zip(task_datapoints, task_arm):
    print ("CURRENT ARM", current_arm)
    # Read lines until you get to the line that you want
    init_point = current_point[0]
    end_point = current_point[1]
    mse_list = []
    angles = []
    human_angles = []
    distances = []
    h_occlussions = []
    r_occlussions = []
    while line_counter < end_point:
        data_point = np.array(skel_reader.readline().split(), dtype=float)
        line_counter += 1
        if line_counter < init_point:
            continue
        sleep(sleep_time)
        human_l = []
        human_r = []
        for l_index, r_index in zip(left_h_joints, right_h_joints):
            human_l.append([-data_point[l_index*3],
                    data_point[l_index*3+1],
                    -data_point[l_index*3+2]])
            human_r.append([-data_point[r_index*3],
                    data_point[r_index*3+1],
                    -data_point[r_index*3+2]])
        human_l = np.array(human_l)*100
        human_r = np.array(human_r)*100
        # get the human arms offset by scale and offset
        l_base = coppelia_pt_to_vpython(np.array(arm_l.base_matrix).astype(np.float64)[:,3]*100)/scale
        r_base = coppelia_pt_to_vpython(np.array(arm_r.base_matrix).astype(np.float64)[:,3]*100)/scale
        offset_human_l = [get_offset_point(p, np.subtract(l_base,human_l[0]), scale) for p in human_l]
        offset_human_r = [get_offset_point(p, np.subtract(r_base,human_r[0]), scale) for p in human_r]
        # clear the previous elements
        for elem_l, elem_r in zip(human_l_chain,human_r_chain):
            elem_l.visible = False
            elem_r.visible = False
        human_l_chain.clear()
        human_r_chain.clear()
        del human_l_chain[:]
        del human_r_chain[:]
        # draw a new element
        human_l_chain = draw_debug(offset_human_l, color.purple,opacity=0.6)
        human_r_chain = draw_debug(offset_human_r, color.purple,opacity=0.6)
        # Get the pose of the new element
        l_constraints =[]
        r_constraints =[]
        for joint_index in range(len(human_l)-1):
            # get the "out" constraint
            out_l_const = get_constraint(human_l[joint_index],human_l[joint_index+1],arm_l.get_base_offsets())
            out_r_const = get_constraint(human_r[joint_index],human_r[joint_index+1],arm_r.get_base_offsets())
            # get the "in" constraint
            in_l_const = get_constraint(human_l[joint_index+1],human_l[joint_index],arm_l.get_base_offsets())
            in_r_const = get_constraint(human_r[joint_index+1],human_r[joint_index],arm_r.get_base_offsets())
            # append to the constraint list
            l_constraints.append(out_l_const +1)
            l_constraints.append(in_l_const +1)
            r_constraints.append(out_r_const +1)
            r_constraints.append(in_r_const +1)
        arm_l.solve(offset_human_l[-1], l_constraints, offset_human_l)
        arm_r.solve(offset_human_r[-1], r_constraints, offset_human_r)
        # print(human_r_chain[-1].po)
        # Get distannce with target
        human_target_l = human_l_chain[-1].pos + human_l_chain[-1].axis
        human_target_r = human_r_chain[-1].pos + human_r_chain[-1].axis
        mse_l = (np.square((arm_l.get_gripper_pos() - human_target_l).value)).mean(axis=None)
        mse_r = (np.square((arm_r.get_gripper_pos() - human_target_r).value)).mean(axis=None)
        if current_arm == 'left':
            mse_list.append([mse_l])
        elif current_arm == 'right':
            mse_list.append([mse_r])
        else:
            mse_list.append([mse_r,mse_l])

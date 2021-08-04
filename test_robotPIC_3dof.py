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
import time
########## PARAMS ##############
# IMPORTANT: the distance units are in
# centimeters for rendering purposes

# example call:
# python incision_task.py -s 3 -t incision_curvy -v 1 -r Baxter -o ./data/results/incision_straight.txt

##############################
###     PARSE ARGS         ###
##############################
iterations = 40
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
left_joint_names = ("lt1, lt2, lt3")
right_joint_names = ("rt1, rt2, rt3")
left_joint_vars = symbols(left_joint_names)
right_joint_vars = symbols(right_joint_names)
neutral = [0,-31,0]
home=[0,0,0]
pose = home

# Define the DH parameter matrix for the Baxter
# following the format:
# alpha(degrees), a(m), d(m), theta(degrees),
# range from(degrees), range to(degrees)
# alpha(degrees), a(m), d(m), theta(degrees),
dh_right = Matrix([
    [90,  0.0084, 0.0834, right_joint_vars[0] + 90],
    [-90, 0.6121,-0.0006, right_joint_vars[1] + 90],
    [0  , 0.5722,-0.1, right_joint_vars[2]]
])

# if the leght of "d" should come before the legth "a"
# when rendering the robots
# TODO: check if this is good
d_first = [1,1,1]

joint_range=np.array([
    [0, 180.0],
    [0, 180.0],
    [0, 180.0]
])


arm_r = robotChain(dh_params=dh_right,joint_vars=right_joint_vars,ranges=joint_range,
        base_matrix=None, pose_imitation=pose_imitation,
        human_joint_index=human_joint_index,
        iterations=iterations, soften=soften, filtering=filtering,
        filter_threshold=filter_threshold,
        render_task=task, render_scale=scale)
arm_r.init_skeleton(pose, d_first)
# create the target
target = sphere(pos=vector(0,0,0),color=color.red, radius = 4)
direction = vector(1,0,0)

#######################################################

def keyInput(keypress):
    global arm_r
    global target
    global scale
    global direction
    rate(100)
    s = keypress.key # get keyboard info
    if s == 's':
        # SOLVE
        r_target = vpython_pt_to_coppelia(np.append(target.pos.value,1))/100
        arm_r.solve(r_target)
    # add to the sphere position
    if s == 'j':
        target.pos = target.pos - direction*scale
    elif s == 'k':
        target.pos = target.pos + direction*scale
    elif s == 'x':
        direction = vector(1,0,0)
    elif s == 'y':
        direction = vector(0,1,0)
    elif s == 'z':
        direction = vector(0,0,1)
    elif e == 'e':
        mse = (np.square((arm_r.get_gripper_pos() - target).value))
        print("squared error for target", mse)

scene.bind('keydown', keyInput)

try:
    while True:
       time.sleep(1)
except KeyboardInterrupt:
    print("Press Ctrl-C to terminate while statement")
    pass

#############################################################################

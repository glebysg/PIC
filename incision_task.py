from vpython import *
from helpers import *
from fabrik import ikLink, ikChain, read_arm
from copy import deepcopy
from datetime import timedelta
import numpy as np
from PIL import Image
from time import sleep
from scipy import integrate
import json
import os
import argparse
import signal
import sys
import threading

# signal handler for killing the program

########## PARAMS ##############
# IMPORTANT: the distance units are in
# centimeters for rendering purposes

# example call:
# python incision_task.py -s 3 -t incision_curvy -v 1 -r Baxter -o ./data/results/incision_straight.txt

##############################
###     PARSE ARGS         ###
##############################
parser = argparse.ArgumentParser()
parser.add_argument('-s', action="store", dest="soften", default=3,
        type=int, help="degree of softening for the imitation algorithm,\
                the valiues can go from 1 to 3")
parser.add_argument('-w', action="store", dest="sleep_time", default=0.02,
        type=float, help="waiting time between datapoint updates ")
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
parser.add_argument('--pose_imitation',action="store_true", default=False,
        help="if present, it uses the pose imitation Algorithm")
parser.add_argument('--append',action="store_true", default=False,
        help="if present, append to the file instead of rewriting it")

args = parser.parse_args()
soften = args.soften
sleep_time = args.sleep_time
robot = args.robot
data_version = args.data_version
task = args.task
pose_imitation = args.pose_imitation
file_append = args.append
skel_path = args.data_path+'smooth_'+task+data_version+'_skel.txt'
ts_path = args.data_path+task+'_skelts.txt'
task_path = args.annotation_path+task+'_'+data_version+'.txt'
pad_path  = "./simulation/textures/pad.jpg"
arm_path = args.config_path+robot+'.txt'
robot_config_path = args.config_path+robot+'_config_'\
        +task+data_version+'.json'
print(task_path)
print(robot_config_path)
task_datapoints = np.loadtxt(task_path, delimiter=' ')
out_file = args.output_path+task+"_"+robot+"_"
# get the algorigthm name for the robot file
algorithm = 'poseimit' if pose_imitation else 'fabrik'
if pose_imitation:
    algorithm += str(soften)
out_file += algorithm+".txt"

###############################

# read robot config
config_reader = open(robot_config_path)
robot_config = json.load(config_reader)
base = robot_config["base"]
human_joint_index = robot_config["human_joint"]
init_constraints = robot_config["constraints"]
scale = robot_config["scale"]
task_arm = robot_config["arm"]
pad_offset = vec(*robot_config["pad_offset"])
pad_axis = vec(*robot_config["pad_axis"])
pad_dim = robot_config["pad_dim"] if task == "assembly" else None

########## Simplified Robot ############################
left_chain, right_chain = read_arm(arm_path)

# Define arm joints
left_h_joints = [4,5,6]
right_h_joints = [8,9,10]

# draw x,y,z
# initialize new bigger canvas
iterations = 20
scene = canvas(title='Pose imitation experiments', width=1200, height=800)
draw_reference_frame(-100,0,100,arrow_size=10)
arm_r = ikChain(chain=right_chain, pose_imitation=pose_imitation,
        human_joint_index=human_joint_index,
        iterations=iterations, soften=soften)
arm_l = ikChain(base=base, chain=left_chain, pose_imitation=pose_imitation,
        human_joint_index=human_joint_index,
        iterations=iterations, soften=soften)
arm_r.init_skeleton(init_constraints=init_constraints)
arm_l.init_skeleton(init_constraints=init_constraints)
arm_r.solve([-10, -70.0, 15.0],init_constraints)
arm_l.solve([60, -70.0, 15.0],init_constraints)

################### incision/assembly pad  ######################
pad_points = []
if task == "assembly":
    length = pad_dim[0]*scale
    height = pad_dim[1]*scale
    width = pad_dim[2]*scale
    pad = box(pos=pad_offset, length=length, height=height, width=width,
            axis=pad_axis,  opacity=0.5, color=color.white)
else:
    length = 25*scale
    height = 2.3*scale
    width = 25*scale
    pad = box(pos=vec(0,0,0), length=length, height=height,
            width=width, texture=pad_path)
########################################################
# Adjust translation and scaling of the human arms.
# For the rotation we have to multiply X and Z by -1
# with open(skel_path, 'r') as skel_file:
    # for data_point in skel_file:
direction = None
human_l_chain = []
human_r_chain = []
skel_reader = open(skel_path, 'r')
line_counter = 0
rate(30)

task_metics = []

for current_point, current_arm in zip(task_datapoints, task_arm):
    # Get pad plane
    # the first point is the 0,0,0 of the pad system
    print(pad_offset)
    pad_points = []
    pad.pos = pad_offset
    pad_points.append((pad.pos+vec(-length, height, -width)/2).value)
    pad_points.append((pad.pos+vec(-length, height, width)/2).value)
    pad_points.append((pad.pos+vec(length, height, width)/2).value)
    pad_points = np.array(pad_points)
    pad_normal = get_plane_normal(pad_points)
    print(pad_points[0])
    # Read lines until you get to the line that you want
    init_point = current_point[0]
    end_point = current_point[1]
    mse_list = []
    angles = []
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
        l_base = np.array(arm_l.base)/scale
        r_base = np.array(arm_r.base)/scale
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
        human_l_chain = draw_debug(offset_human_l, color.yellow,opacity=0.5)
        human_r_chain = draw_debug(offset_human_r, color.yellow,opacity=0.5)
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
        arm_l.solve(offset_human_l[-1], l_constraints)
        arm_r.solve(offset_human_r[-1], r_constraints)
        # print(human_r_chain[-1].po)
        # Get distannce with target
        human_target_l = human_l_chain[-1].pos + human_l_chain[-1].axis
        human_target_r = human_r_chain[-1].pos + human_r_chain[-1].axis
        mse_l = (np.square((arm_l.gripper.pos - human_target_l).value)).mean(axis=None)
        mse_r = (np.square((arm_r.gripper.pos - human_target_r).value)).mean(axis=None)
        if current_arm == 'left':
            mse_list.append([mse_l])
        elif current_arm == 'right':
            mse_list.append([mse_r])
        else:
            mse_list.append([mse_r,mse_l])
        # Get the Pose
        ## Human
        ### Get the shoulder link pointing towards the shoulder
        shoulder_h_l = vec(*(offset_human_l[0]-offset_human_l[1]))
        shoulder_h_r = vec(*(offset_human_r[0]-offset_human_r[1]))
        ### Get the elbow link pointing towards the wrist
        elbow_h_l = vec(*(offset_human_l[2]-offset_human_l[1]))
        elbow_h_r = vec(*(offset_human_r[2]-offset_human_r[1]))
        ### Get the angle
        angle_h_l = diff_angle(shoulder_h_l,elbow_h_l)
        angle_h_r = diff_angle(shoulder_h_r,elbow_h_r)
        ### Get the distance between shoulder and wrist
        dist_h_l = np.linalg.norm((shoulder_h_l-elbow_h_l).value)
        dist_h_r = np.linalg.norm((shoulder_h_r-elbow_h_r).value)
        ###### Robot
        angle_r_l, dist_r_l = get_robot_angle(arm_l, human_joint_index)
        angle_r_r, dist_r_r = get_robot_angle(arm_r, human_joint_index)
        # Append the squared differences in angles and distances
        angles.append((angle_h_l-angle_r_l)**2)
        angles.append((angle_h_r-angle_r_r)**2)
        distances.append((dist_h_l-dist_r_l)**2)
        distances.append((dist_h_r-dist_r_r)**2)
        ####################################################
        ############ get the human occluded area ########################
        pad_origin = pad_points[0]
        h_elbow = project_to_plane(pad_normal,pad_origin,offset_human_r[1])
        h_wrist = project_to_plane(pad_normal,pad_origin,offset_human_r[2])
        h_elbow = h_elbow - pad_origin
        h_wrist = h_wrist - pad_origin
        ####### only calculate if the wrist was under the pad
        if h_wrist[0] > 0 and h_wrist[0] < length and\
                h_wrist[2] > 0 and h_wrist[2] < width:
            ############ get the human occluded area calculation ###########
            h_m, h_b= get_line([h_elbow[0],h_elbow[2]],[h_wrist[0],h_wrist[2]])
            h_line = lambda x: h_m*x + h_b
            x_points = [h_elbow[0],h_wrist[0]]
            h_occlussion_count = 0
            for x in range(0,int(length)):
                for z in range(0, int(width)):
                    if is_above_line([x,z], h_line) and\
                        x >= min(x_points) and x <= max(x_points):
                        h_occlussion_count += 1
            # print(h_occlussion_count,h_occluded_area)
            h_occluded_area = h_occlussion_count/float(length*width)
            ############ get the robot occluded area ########################
            # Get the offset between the human and the robot at the wrist
            h_r_offset = np.array(arm_r.points[-1]) - np.array(offset_human_r[-1])
            pad_origin = pad_points[0] + h_r_offset
            # get the area under the curve for each joint
            elbow_index = human_joint_index[1]
            r_occlussion_count = 0
            for joint_i in range(elbow_index, len(arm_r.points)-1):
                joint_1 = project_to_plane(pad_normal,pad_origin,arm_r.points[joint_i])
                joint_2 = project_to_plane(pad_normal,pad_origin,arm_r.points[joint_i+1])
                joint_1 = joint_1 - pad_origin
                joint_2 = joint_2 - pad_origin
                ############ robot occluded area calculation ########################
                r_m, r_b= get_line([joint_1[0],joint_1[2]],[joint_2[0],joint_2[2]])
                if r_m is None:
                    continue
                h_line = lambda x: r_m*x + r_b
                for x in range(0,int(length)):
                    for z in range(0, int(width)):
                        if is_above_line([x,z], h_line) and\
                            x >= min(x_points) and x <= max(x_points):
                            r_occlussion_count += 1
            r_occluded_area = r_occlussion_count/float(length*width)
            ############ Append the occluded areas ######################
            h_occlussions.append(h_occluded_area)
            r_occlussions.append(r_occluded_area)
    mse_list = np.array(mse_list)
    angles= np.array(angles)
    # if the angle is less than 5 degrees
    angles_mask = (angles <= 0.087).astype(int)
    pose_percentage = sum(angles_mask)/float(len(angles_mask))
    print(robot, task, "soften:", soften, "PI:", pose_imitation)
    print("POSE PERCENTAGE %.2f" % pose_percentage)
    print("HUMAN OCCLUDED AREA %.3f" % np.mean(h_occlussions))
    print("ROBOT OCCLUDED AREA %.3f" % np.mean(r_occlussions))
    print("MEAN SQUARED ERROR:", str(round(np.mean(mse_list),2)))
    task_metics.append([pose_percentage, np.mean(h_occlussions), np.mean(r_occlussions), np.mean(mse_list)])
    # angles= np.mean(angles/max(angles))
    # distances= np.array(distances)
    # distances= np.mean(distances/max(distances))
    # print("POSE ANGLE:", str(round(angles, 2)))
    # print("POSE DISTANCES:", str(round(distances, 2)))
    # print("POSE F1:", str(round(2*(angles*distances)/(angles+distances),2)))

task_metics = np.array(task_metics)
if file_append:
    old_metrics = np.loadtxt(out_file, delimiter=' ')
    print(old_metrics)
    task_metics = np.concatenate((old_metrics, task_metics), axis=0)
    print(task_metics)
np.savetxt(out_file, task_metics, delimiter=' ')

scene.delete()
print("DONE!")

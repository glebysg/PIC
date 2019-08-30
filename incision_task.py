from vpython import *
from helpers import *
from fabrik import ikLink, ikChain, read_arm
from copy import deepcopy
from datetime import timedelta
import numpy as np
from PIL import Image
from time import sleep
import json
import os

current_dir = os.getcwd()
########## PARAMS ##############
# IMPORTANT: the distance units are in
# centimeters for rendering purposes
soften = 3
robot = "baxter"
data_version = '1'
task = 'incision'
pose_imitation = False
skel_path = './data/smooth_'+task+data_version+'_skel.txt'
ts_path = './data/'+task+'_skelts.txt'
task_path = './simulation/data_points/'+task+'_'+data_version+'.txt'
pad_path  = "./simulation/textures/pad.jpg"
skel_description = './data/'
arm_path = './simulation/arms/'+robot+'.txt'
robot_config_path = './simulation/arms/'+robot+'_config_'\
        +task+data_version+'.json'
task_datapoints = np.loadtxt(task_path, delimiter=' ')
###############################

# read robot config
config_reader = open(robot_config_path)
robot_config = json.load(config_reader)
base = robot_config["base"]
human_joint_index = robot_config["human_joint"]
init_constraints = robot_config["constraints"]
offset = vec(*robot_config["offset"])
pad_offset = vec(*robot_config["pad_offset"])
scale = robot_config["scale"]
task_arm = robot_config["arm"]

########## Simplified Robot ############################
left_chain, right_chain = read_arm(arm_path)

# Define arm joints
left_h_joints = [4,5,6]
right_h_joints = [8,9,10]

# draw x,y,z
# initialize new bigger canvas
# scene = canvas(title='Coaching Gym', width=1200, height=800)
draw_reference_frame(-100,0,100,arrow_size=10)
arm_r = ikChain(chain=right_chain, pose_imitation=pose_imitation,
        human_joint_index=human_joint_index,
        iterations=20, soften=soften)
arm_l = ikChain(base=base, chain=left_chain, pose_imitation=pose_imitation,
        human_joint_index=human_joint_index,
        iterations=20, soften=soften)
arm_r.init_skeleton(init_constraints=init_constraints)
arm_l.init_skeleton(init_constraints=init_constraints)
arm_r.solve([-10, -70.0, 15.0],init_constraints)
arm_l.solve([60, -70.0, 15.0],init_constraints)

################### incision pad  ######################
length = 30*scale
height = 3*scale
width = 15*scale
pad_points = []
pad = box(pos=pad_offset, length=length, height=height,
        width=width, texture=pad_path)
# Get pad plane
# the first point is the 0,0,0 of the pad system
pad_points.append((pad.pos+vec(-length, height, -width)/2).value)
pad_points.append((pad.pos+vec(-length, height, width)/2).value)
pad_points.append((pad.pos+vec(length, height, width)/2).value)
pad_points = np.array(pad_points)
p_x, p_y, p_z, p_d = get_plane(pad_points)
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

for current_point, current_arm in zip(task_datapoints, task_arm):
    # Read lines until you get to the line that you want
    init_point = current_point[0]
    end_point = current_point[1]
    mse_list = []
    angles = []
    distances = []
    while line_counter < end_point:
        data_point = np.array(skel_reader.readline().split(), dtype=float)
        line_counter += 1
        if line_counter < init_point:
            continue
        # sleep(0.02)
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
        # clear the previous elements
        for elem_l, elem_r in zip(human_l_chain,human_r_chain):
            elem_l.visible = False
            elem_r.visible = False
        human_l_chain.clear()
        human_r_chain.clear()
        del human_l_chain[:]
        del human_r_chain[:]
        # draw a new element
        human_l_chain = draw_debug(human_l*scale, color.yellow,opacity=0.5)
        human_r_chain = draw_debug(human_r*scale, color.yellow,opacity=0.5)
        for elem_l, elem_r in zip(human_l_chain,human_r_chain):
            elem_l.pos = (elem_l.pos + offset*scale)
            elem_r.pos = (elem_r.pos + offset*scale)
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
        arm_l.solve(human_l[-1]*scale + np.array(offset.value)*scale, l_constraints)
        arm_r.solve(human_r[-1]*scale + np.array(offset.value)*scale, r_constraints)
        # Get distannce with target
        human_target_l = human_l_chain[-1].pos + human_l_chain[-1].axis
        human_target_r = human_r_chain[-1].pos + human_r_chain[-1].axis
        mse_l = (np.square((arm_l.gripper.pos - human_target_l).value)).mean(axis=None)
        mse_r = (np.square((arm_r.gripper.pos - human_target_r).value)).mean(axis=None)
        mse_list.append([mse_r,mse_l])
        # Get the Pose
        ## Human
        ### Get the shoulder link pointing towards the shoulder
        shoulder_h_l = vec(*(human_l[0]-human_l[1]))
        shoulder_h_r = vec(*(human_r[0]-human_r[1]))
        ### Get the elbow link pointing towards the wrist
        elbow_h_l = vec(*(human_l[2]-human_l[1]))
        elbow_h_r = vec(*(human_r[2]-human_r[1]))
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
        # Get area under incision
        elbow_index = human_joint_index[1]
        # for j_index in range(elbow_index, len(arm_r.points)):

    mse_list = np.array(mse_list)
    angles= np.array(angles)
    # if the angle is less than 5 degrees
    angles_mask = (angles <= 0.087).astype(int)
    pose_percentage = sum(angles_mask)/float(len(angles_mask))
    print(robot, task, "soften:", soften, "PI:", pose_imitation)
    print("POSE PERCENTAGE %.2f" % pose_percentage)

    # angles= np.mean(angles/max(angles))
    # distances= np.array(distances)
    # distances= np.mean(distances/max(distances))
    # print("MEAN SQUARED ERROR:", str(round(np.mean(mse_list),2)))
    # print("POSE ANGLE:", str(round(angles, 2)))
    # print("POSE DISTANCES:", str(round(distances, 2)))
    # print("POSE F1:", str(round(2*(angles*distances)/(angles+distances),2)))

exit()

    # Get shoulder wrist distance
    ### For the human arms
    ### For the robot

# chain.solve([-83.8738,34.6046, -2.1450], init_constraints)
# chain.animate()
# while True:
    # chain.animate()

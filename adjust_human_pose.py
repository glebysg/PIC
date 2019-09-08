from vpython import *
from helpers import draw_reference_frame, draw_debug
from fabrik import ikLink, ikChain, read_arm
from copy import deepcopy
from datetime import timedelta
import numpy as np
import json

########## PARAMS ##############
soften = 3
robot = "baxter"
data_version = ''
task = 'incision_curvy'
pose_imitation = True
skel_path = './data/new/smooth_'+task+data_version+'_skel.txt'
ts_path = './data/new/'+task+data_version+'_skelts.txt'
skel_description = './data/new/'
arm_path = './simulation/arms/'+robot+'.txt'
robot_config_path = './simulation/arms/'+robot+'_config_'\
                    +task+data_version+'.json'
ignore_secs = 90 
# ignore_secs = 0
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


########## Simplified robot ############################
left_chain, right_chain = read_arm(arm_path)

# Ignore the first <ignore_seconds> seconds
line_counter = 0
first = True
first_ts = None
with open(ts_path, "r") as timestamps:
    for ts in timestamps:
        if first:
            first_ts = timedelta(seconds=float(ts))
            first = False
        if timedelta(seconds=float(ts)) - first_ts > timedelta(seconds=ignore_secs):
            break
        line_counter += 1

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


############# assebly pieces ###########################
pad_path  = "./simulation/textures/pad.jpg"
length = 25*scale
height = 2.3*scale
width = 25*scale
pad = box(pos=pad_offset, length=length, height=height,
        width=width, texture=pad_path)
########################################################

# Adjust translation and scaling of the human arms.
# For the rotation we have to multiply X and Z by -1
# with open(skel_path, 'r') as skel_file:
    # for data_point in skel_file:
direction = None
human_l = []
human_r = []
human_l_chain = []
human_r_chain = []
skel_reader = open(skel_path, 'r')
robot = True
data_count = line_counter

for i in range(line_counter):
    skel_reader.readline()

def keyInput(keypress):
    global direction
    global human_l
    global human_r
    global human_l_chain
    global human_r_chain
    global datapoint
    global left_h_joints
    global right_h_joints
    global offset
    global pad_offset
    global scale
    global robot
    global data_count
    rate(30)
    s = keypress.key # get keyboard info
    if len(s) == 1:
        if s == 'r':
            robot = not robot
        if s == 'x':
                direction = vec(1,0,0)
        elif s == 'y':
            direction = vec(0,1,0)
        elif s == 'z':
            direction = vec(0,0,1)
        elif direction is not None \
            and (s == 'l' or s == 'i'):
            if robot:
                offset += direction
                for elem_l, elem_r in zip(human_l_chain,human_r_chain):
                    elem_l.pos = elem_l.pos + direction*scale
                    elem_r.pos = elem_r.pos + direction*scale
            else:
                pad_offset += direction*scale
                pad.pos = pad.pos + direction*scale
        elif direction is not None \
            and (s == 'j' or s == 'k'):
            if robot:
                offset -= direction
                for elem_l, elem_r in zip(human_l_chain,human_r_chain):
                    elem_l.pos = elem_l.pos - direction*scale
                    elem_r.pos = elem_r.pos - direction*scale
            else:
                pad_offset -= direction*scale
                pad.pos = pad.pos - direction*scale
        elif s == 'n' or s== 'b' or s=='s':
            if s == 'n':
                data_count += 1
                data_point = np.array(skel_reader.readline().split(), dtype=float)
                human_l = []
                human_r = []
                for l_index, r_index in zip(left_h_joints, right_h_joints):
                    human_l.append([-data_point[l_index*3],
                            data_point[l_index*3+1],
                            -data_point[l_index*3+2]])
                    human_r.append([-data_point[r_index*3],
                            data_point[r_index*3+1],
                            -data_point[r_index*3+2]])
                human_l = np.array(human_l)
                human_r = np.array(human_r)
            elif s == 'b':
                scale += 0.1
                pad.width = width*scale
                pad.height = height*scale
                pad.length = length*scale
            elif s == 's':
                scale -= 0.1
                pad.width = width*scale
                pad.height = height*scale
                pad.length = length*scale
            # clear the previous elements
            for elem_l, elem_r in zip(human_l_chain,human_r_chain):
                elem_l.visible = False
                elem_r.visible = False
            human_l_chain.clear()
            human_r_chain.clear()
            del human_l_chain[:]
            del human_r_chain[:]
            # draw a new element
            human_l_chain = draw_debug(human_l*100*scale, color.yellow)
            human_r_chain = draw_debug(human_r*100*scale, color.yellow)
            for elem_l, elem_r in zip(human_l_chain,human_r_chain):
                elem_l.pos = (elem_l.pos + offset*scale)
                elem_r.pos = (elem_r.pos + offset*scale)
        elif s == 'p':
            print("Offset:", offset)
            print("Pad Offset:", pad_offset)
            print("Scale:", scale)
            print("Datapoint:", data_count)



scene.bind('keydown', keyInput)



# chain.solve([-83.8738,34.6046, -2.1450], init_constraints)
# chain.animate()
# while True:
    # chain.animate()

from vpython import *
from helpers import draw_reference_frame, draw_debug
from fabrik import ikLink, ikChain
from copy import deepcopy
from datetime import timedelta
import numpy as np

########## PARAMS ##############
soften = 3
pose_imitation = False
human_joint_index = [0,2,4]
init_constraints = [8,3,7,1]
###############################
right_chain = []

########## Simplified baxter ############################

right_chain.append(ikLink(length=6.9,orientation=[-1,0,0]))
right_chain.append(ikLink(length=36.435,orientation=[-1,0,0]))
right_chain.append(ikLink(length=37.429,orientation=[-1,0,0]))
right_chain.append(ikLink(length=36.830,orientation=[-1,0,0]))
########################################################

# Ignore the first <ignore_seconds> seconds
# draw x,y,z
# initialize new bigger canvas
# scene = canvas(title='Coaching Gym', width=1200, height=800)
draw_reference_frame(-100,0,100,arrow_size=10)
arm_r = ikChain(chain=right_chain, pose_imitation=pose_imitation,
        human_joint_index=human_joint_index,
        iterations=20, soften=soften)
arm_r.init_skeleton(init_constraints=init_constraints)
arm_r.solve([-10, -70.0, 15.0],init_constraints)

# Adjust translation and scaling of the human arms.
# For the rotation we have to multiply X and Z by -1
# with open(skel_path, 'r') as skel_file:
    # for data_point in skel_file:

# chain.solve([-83.8738,34.6046, -2.1450], init_constraints)
# chain.animate()
# while True:
    # chain.animate()

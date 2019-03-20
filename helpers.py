from vpython import *
import math
import itertools

def draw_reference_frame(x,y,z,arrow_size=5):
    arrow(pos=vector(x,y,z), axis=vector(arrow_size,0,0), shaftwidth=1, color=color.red)
    arrow(pos=vector(x,y,z), axis=vector(0,arrow_size,0), shaftwidth=1, color=color.green)
    arrow(pos=vector(x,y,z), axis=vector(0,0,arrow_size), shaftwidth=1, color=color.blue)
    text(text='X', align='center', height=5, depth=-0.3, color=color.red,pos=vector(x+arrow_size+1,y,z))
    text(text='Y', align='center', height=5, depth=-0.3, color=color.green,pos=vector(x,y+arrow_size+1,z))
    text(text='Z', align='center', height=5, depth=-0.3, color=color.blue,pos=vector(x,y,z+arrow_size+1))

def is_constraint_intersection(joint_center, offset_matrix, constraint_index, target_center):
    # get the angle between the center and the constraint vector
    constraint_angle = None
    min_offset_angle = math.pi
    for index in len(offset_matrix):
        offset = offset_matrix[index]

        # get the constraint vector
        # if we are using 8 cubes, the vectors would point at 45 degrees
        # towards  the opposite corner from the center

        # Get the angle between the target joint
        # and the target constraint
        constraint_vec =  joint_center + vec(*offset)
        offset_angle = diff_angle(constraint_angle, target_center)
        if index == constraint_index:
            constraint_angle = offset_angle
        # Get the smallest angle between the other constraints
        # and the target joint
        elif offset_angle < min_offset_angle:
            min_offset_angle = offset_angle

        # if the angle between the target joint and the target constraint
        # is the smallest, the link intersects with the constraint
        if constraint_angle > min_offset_angle:
            return True
        else:
            return False

def get_projection(joint_center,constraint_offset, target_center):
    combinations = itertools.combinations(range(3),2)
    min_angle_with_corner = math.pi*2
    min_corner = None
    for i,j in combinations:
        # get constraint corner that is closer to the target
        offset = [0,0,0]
        offset[i] += constraint_offset[i]
        offset[j] += constraint_offset[j]
        corner = joint_center + vec(*offset)
        angle = diff_angle(corner, target_center)
        if angle < min_offset_angle:
            min_offset_angle = angle
            min_corner = offset
    # project the line onto that corner




    pass







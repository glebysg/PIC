from vpython import *

def draw_reference_frame(x,y,z,arrow_size=5):
    arrow(pos=vector(x,y,z), axis=vector(arrow_size,0,0), shaftwidth=1, color=color.red)
    arrow(pos=vector(x,y,z), axis=vector(0,arrow_size,0), shaftwidth=1, color=color.green)
    arrow(pos=vector(x,y,z), axis=vector(0,0,arrow_size), shaftwidth=1, color=color.blue)
    text(text='X', align='center', height=5, depth=-0.3, color=color.red,pos=vector(x+arrow_size+1,y,z))
    text(text='Y', align='center', height=5, depth=-0.3, color=color.green,pos=vector(x,y+arrow_size+1,z))
    text(text='Z', align='center', height=5, depth=-0.3, color=color.blue,pos=vector(x,y,z+arrow_size+1))

def is_constraint_intersection(joint_center, offset_matrix, constraint_index):
    # get the angle between the center and the constraint vector
    for index in len(offset_matrix):
        offset = offset_matrix[index]
        # get the constraint vector
        # if we are using 8 cubes, the vectors would point at 45 degrees
        # towards  the opposite corner from the center
        constraint_vec =  joint_center + vec(*offset)
        if index == constraint_index:
            pass

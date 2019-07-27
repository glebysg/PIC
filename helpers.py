from vpython import *
import math
import itertools
import numpy as np

def normalize(vector):
    return np.array(vector,dtype=float)/np.linalg.norm(vector)

def distance(vector1, vector2):
    if type(vector1).__name__ == 'vector':
        return np.linalg.norm(np.array((vector1-vector2).value))
    else:
        return np.linalg.norm(vector1-vector2)

def vec_to_np(vector, dtype):
    return np.array([
      vector.x,
      vector.y,
      vector.z,
      ], dtype=dtype)

def draw_reference_frame(x,y,z,arrow_size=5):
    arrow(pos=vector(x,y,z), axis=vector(arrow_size,0,0), shaftwidth=1, color=color.red)
    arrow(pos=vector(x,y,z), axis=vector(0,arrow_size,0), shaftwidth=1, color=color.green)
    arrow(pos=vector(x,y,z), axis=vector(0,0,arrow_size), shaftwidth=1, color=color.blue)
    text(text='X', align='center', height=5, depth=-0.3, color=color.red,pos=vector(x+arrow_size+1,y,z))
    text(text='Y', align='center', height=5, depth=-0.3, color=color.green,pos=vector(x,y+arrow_size+1,z))
    text(text='Z', align='center', height=5, depth=-0.3, color=color.blue,pos=vector(x,y,z+arrow_size+1))

def is_constraint_intersection(joint_center, offset_matrix, constraint_index, target):
    # turn the points into vectors
    target = vec(*target)
    joint_center = vec(*joint_center)
    # get the angle between the target and the constraint of interest
    # get the constraint vector
    # if we are using 8 cubes, the vectors would point at 45 degrees
    # towards  the opposite corner from the center
    offset = offset_matrix[constraint_index]
    constraint_vec =  joint_center + vec(*offset)
    # Get the angle between the target joint
    # and the target constraint
    target_angle = diff_angle(constraint_vec, target)
    for index in range(len(offset_matrix)):
        offset = offset_matrix[index]
        constraint_vec =  joint_center + vector(*offset)
        constraint_angle = diff_angle(constraint_vec, target)
        # if we find a smaller angle, the link does not intersect
        # with the constraint
        if constraint_angle < target_angle:
            return False
    return True

def get_projection(joint_center,offset_matrix, constraint_index, target, tolerance=0):
    # turn the points into vectors
    target = vec(*target)
    joint_center = vec(*joint_center)
    combinations = itertools.combinations(range(3),2)
    min_offset_angle = math.pi*2
    min_corner = None
    joint_constraint = offset_matrix[constraint_index]
    constraint_offset = joint_constraint
    # Find the constraint that is closest to the target,
    # given the current tolerance
    min_angle = diff_angle(joint_center +vec(*joint_constraint), target)
    if tolerance > 0:
        for offset in offset_matrix:
            # skip the neighboors that are not within
            # the tolerance level
            if sum(abs((offset - joint_constraint))) > tolerance*2:
                continue
            constraint_vec =  joint_center + vector(*offset)
            angle = diff_angle(constraint_vec, target)
            if  angle < min_angle:
                constraint_offset = offset
                min_angle = angle

    for i,j in combinations:
        # get constraint corner that is closer to the target
        offset = [0,0,0]
        offset[i] = constraint_offset[i]
        offset[j] = constraint_offset[j]
        corner = joint_center + vec(*offset)
        angle = diff_angle(corner, target)
        if angle < min_offset_angle:
            min_offset_angle = angle
            min_corners = (i,j)
    # project the line onto that corner
    # First find the normal to the plane to project
    # rebase target at joint center.
    target = target - joint_center
    c1 = [0,0,0]
    c1[min_corners[0]] = constraint_offset[min_corners[0]]
    c1 = vec(*c1)
    c2 = [0,0,0]
    c2[min_corners[1]] = constraint_offset[min_corners[1]]
    c2 = vec(*c2)
    normal = c1.cross(c2)
    proj = target - (target.dot(normal)/mag2(normal))*normal
    # check if the projection is inside the target constraint
    if (c1.cross(proj)).dot(c1.cross(c2)) >= 0 \
        and (c2.cross(proj)).dot(c2.cross(c1)) >=0:
        return proj.value#, normal, c1, c2
    # if it's not, get the closest edge vector and use it
    # as the new orientation
    else:
        if diff_angle(c1, target) < diff_angle(c2,target):
            return c1.value#, normal, c1, c2
        else:
            return c2.value#, normal, c1, c2

def get_constraint(center, target, constraint_matrix):
    center = vec(*center)
    link = vec(*target) - center
    min_angle = math.pi*2
    constraint_index = None
    for i in range(len(constraint_matrix)):
        constraint_vec =  center + vector(*constraint_matrix[i])
        angle = diff_angle(constraint_vec, link)
        if angle < min_angle:
            constraint_index = i
            min_angle = angle
    return constraint_index

def draw_debug(points,color,opacity=1):
    axis = None
    # self.ik_sphere.pos = vec(*self.target)
    # self.ik_sphere.radius = 5
    chain = []
    for index in range(len(points)-1):
        # Normalize the orientation o f the ik link
        pos = vec(*points[index])
        length = distance(points[index],points[index+1])
        orientation = normalize(points[index+1]-points[index])
        axis = vec(*(orientation*length))
        joint =  sphere(pos=pos,color=color, radius = 4, opacity=opacity)
        link = cylinder(pos=pos, axis=axis, color=color,radius=2,opacity=opacity)
        chain.append(joint)
        chain.append(link)
    return chain

# The main function is used to test the helper functions
def main():
    offsets = [[-1,1,-1],[1,1,-1],
        [1,1,1],[-1,1,1],[-1,-1,-1],
        [1,-1,-1],[1,-1,1],[-1,-1,1]]
    center = vector(0,0,0)
    targets = [vector(-2,3,3),
            vector(-2,3,0),
            vector(-1,3,-1)]
    c_index = 3
    print("The answers for intersection  should be True, True, False")
    for target in targets:
        print(is_constraint_intersection(center, offsets, c_index, target))

def get_robot_angle(chain, joint_index):
    shoulder_joint = chain.points[joint_index[0]]
    elbow_joint = chain.points[joint_index[1]]
    wrist_joint = chain.points[joint_index[2]]
    # Get the shoulder link pointing towards the shoulder
    shoulder = vec(*(shoulder_joint-elbow_joint))
    # Get the elbow link pointing towards the wrist
    elbow = vec(*(wrist_joint - elbow_joint))
    angle = diff_angle(shoulder,elbow)
    pose_dist = np.linalg.norm((shoulder - elbow).value)
    return angle, pose_dist

if __name__ == "__main__":
    # execute only if run as a script
    main()








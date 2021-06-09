from vpython import *
import math
import itertools
import numpy as np
from sympy import *

def normalize(vector):
    l2 = np.linalg.norm(vector)
    if l2 == 0:
        return(np.zeros(len(vector)))
    return np.array(vector,dtype=float)/np.linalg.norm(vector)

def distance(vector1, vector2):
    if type(vector1).__name__ == 'vector' and type(vector2).__name__ == 'vector':
        return np.linalg.norm(np.array((vector1-vector2).value))
    # transform the vector 2 to vector
    elif type(vector1).__name__ == 'vector':
        return np.linalg.norm(vector1.value-vector2)
    # transform the vector 1 to vector
    elif type(vector2).__name__ == 'vector':
        return np.linalg.norm(vector1-vector2.value)
    else:
        return np.linalg.norm(vector1-vector2)

def vec_to_np(vector, dtype):
    return np.array([
      vector.x,
      vector.y,
      vector.z,
      ], dtype=dtype)

def coppelia_pt_to_vpython(coppelia_pt):
    try:
        if coppelia_pt.size<4:
            coppelia_pt = np.append(coppelia_pt,1)
            print("transformed pt", coppelia_pt)
    except:
        print("untransformed pt", coppelia_pt)
    rot = np.array([
        [0,1,0,0],
        [0,0,1,0],
        [1,0,0,0],
        [0,0,0,1]
        ])
    # Remove the homogeneous coordinate element from the vector
    return np.dot(rot,coppelia_pt)[:-1]


def vpython_pt_to_coppelia(vpython_pt):
    rot = np.array([
        [0,0,1,0],
        [1,0,0,0],
        [0,1,0,0],
        [0,0,0,1]
        ])
    # Remove the homogeneous coordinate element from the vector
    return np.dot(rot,vpython_pt)[:-1]

def coppelia_frame_to_vpython(coppelia_frame):
    rot = np.array([
        [0,1,0,0],
        [0,0,1,0],
        [1,0,0,0],
        [0,0,0,1]
        ])
    # Remove the homogeneous coordinate element from the vector
    return np.dot(rot,coppelia_frame)

def draw_vpython_reference_frame(x,y,z,arrow_size=5):
    arrow(pos=vector(x,y,z), axis=vector(arrow_size,0,0), shaftwidth=1, color=color.red)
    arrow(pos=vector(x,y,z), axis=vector(0,arrow_size,0), shaftwidth=1, color=color.green)
    arrow(pos=vector(x,y,z), axis=vector(0,0,arrow_size), shaftwidth=1, color=color.blue)
    text(text='X', align='center', height=5, depth=-0.3, color=color.red,pos=vector(x+arrow_size+1,y,z))
    text(text='Y', align='center', height=5, depth=-0.3, color=color.green,pos=vector(x,y+arrow_size+1,z))
    text(text='Z', align='center', height=5, depth=-0.3, color=color.blue,pos=vector(x,y,z+arrow_size+1))

def draw_coppelia_reference_frame(x,y,z,arrow_size=5):
    arrow(pos=vector(x,y,z), axis=vector(arrow_size,0,0), shaftwidth=1, color=color.green)
    arrow(pos=vector(x,y,z), axis=vector(0,arrow_size,0), shaftwidth=1, color=color.blue)
    arrow(pos=vector(x,y,z), axis=vector(0,0,arrow_size), shaftwidth=1, color=color.red)
    text(text='Y', align='center', height=5, depth=-0.3, color=color.green,pos=vector(x+arrow_size+1,y,z))
    text(text='Z', align='center', height=5, depth=-0.3, color=color.blue,pos=vector(x,y+arrow_size+1,z))
    text(text='X', align='center', height=5, depth=-0.3, color=color.red,pos=vector(x,y,z+arrow_size+1))

def draw_reference_frame(rotation,arrow_size=10, transform=True):
    # the position is the translation part
    if transform:
        pos = vector(*((coppelia_pt_to_vpython(rotation[:,3])*100)))
        rot = coppelia_frame_to_vpython(rotation)
    else:
        rot = rotation
        pos = vector(*(rot[0:3,3]*100))
    x_axis = norm(vector(*rot[0:3,0]))*arrow_size
    y_axis = norm(vector(*rot[0:3,1]))*arrow_size
    z_axis = norm(vector(*rot[0:3,2]))*arrow_size
    x = arrow(pos=pos, axis=x_axis, shaftwidth=1, color=color.red)
    y = arrow(pos=pos, axis=y_axis, shaftwidth=1, color=color.green)
    z = arrow(pos=pos, axis=z_axis, shaftwidth=1, color=color.blue) 
    return [x,y,z]

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
    returnpTrue

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

def is_conic_intersection(center, target, axis, angle):
    center = vec(*center)
    target = vec(*target)
    constraint_axis = center + axis
    robot_axis = target - center
    if diff_angle(constraint_axis, robot_axis) < np.radians(angle/2):
        return True
    else:
        return False

def get_conic_projection(r_joint, r_target, h_axis, angle):
    r_axis = vec(*r_target) - vec(*r_joint)
    # get the normal to the plane formed between
    # the robot and the translated human axis
    norm_rh = cross(r_axis, h_axis).norm()
    # rotate the human axis +- theta degrees about
    # the normal using the rodrigez formula
    rot_plus = rotate(h_axis, angle=np.radians(angle/2),
            axis=norm_rh)
    rot_minus = rotate(h_axis, angle=np.radians(-angle)/2,
            axis=norm_rh)
    # get which rotated human axis is closer to the robot axis
    # and return the closest
    if diff_angle(r_axis, rot_plus) < diff_angle(r_axis, rot_minus):
        return rot_plus.value
    else:
        return rot_minus.value

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

# receives a nxm array of points,
# where n = m = 3 and n is the number
# of points and m is the number of dimentions (3)
def get_plane_normal(points):
    p1 = np.array([1, 2, 3])
    p2 = np.array([4, 6, 9])
    p3 = np.array([12, 11, 9])
    # These two vectors are in the plane
    v1 = points[2] - points[0]
    v2 = points[1] - points[0]
    # the cross product is a vector normal to the plane
    cp = np.cross(v1, v2)
    return cp

def project_to_plane(normal,plane_point,point):
    x = point[0]
    y = point[1]
    z = point[2]
    a = normal[0]
    b = normal[1]
    c = normal[2]
    d = plane_point[0]
    e = plane_point[1]
    f = plane_point[2]
    plane_norm = a**2 + b**2 + c**2
    if plane_norm == 0:
        return np.zeros(3)
    t = (a*d - a*x + b*e - b*y + c*f -c*z)/\
       plane_norm
    x_proy = x + t*a
    y_proy = y + t*b
    z_proy = z + t*c
    return np.array([x_proy,y_proy,z_proy])

def pt_project_to_plane(p1, p2, p3, p_point):
    norm =  np.linalg.norm(np.cross(p2-p1,p3-p2))
    if norm == 0:
        return zeros(3)
    n = np.cross(p2-p1,p3-p2)/norm
    print("project_to_plane", p_point-(np.dot(p_point-p1,n))*n)
    return p_point-(np.dot(p_point-p1,n))*n


def get_line(p,q):
    # make the line not completely straight to get some
    # arm occlussion when the arm is straight
    if (q[0] - p[0]) == 0:
        p[0]+=1
    m = (q[1] - p[1])/(q[0] - p[0])
    b = p[1]-m*p[0]
    return m,b
# get a point offset by <offset and scaled>
# by <scale>
def get_offset_point(point, offset, scale):
    return np.array(point)*scale + np.array(offset)*scale

def is_above_line(point, line):
    x = point[0]
    y = point[1]
    return y > line(x)

def main():
    print(get_line([2,4],[-1,1]))
    print(get_line([2,4],[0,2]))
    print("the answer should be %d, %d" % (1,2))

# Unidimentional Kalman filter for smoothing,
# where data is the input and Q is the variance
def kalman_filter(data, Q):
    n_iter = len(data)
    sz = (n_iter,) # size of array

    # allocate space for arrays
    xhat=np.zeros(sz)      # a posteri estimate of x
    P=np.zeros(sz)         # a posteri error estimate
    xhatminus=np.zeros(sz) # a priori estimate of x
    Pminus=np.zeros(sz)    # a priori error estimate
    K=np.zeros(sz)         # gain or blending factor
    R = 0.1**2 # estimate of measurement variance, change to see effect
    # intial guesses
    xhat[0] = 0.0
    P[0] = 1.0

    for k in range(1,n_iter):
        # time update
        xhatminus[k] = xhat[k-1]
        Pminus[k] = P[k-1]+Q
        # measurement update
        K[k] = Pminus[k]/( Pminus[k]+R )
        xhat[k] = xhatminus[k]+K[k]*(data[k]-xhatminus[k])
        P[k] = (1-K[k])*Pminus[k]

    # return the estimated variable
    return xhat, Pminus

def wrist_under_occlusion_area(h_wrist,pad_dim,pad_axis,scale):
    length = pad_dim[0]*scale
    height = pad_dim[1]*scale
    width = pad_dim[2]*scale
    # if the occlussion/pad is in the x-z plane
    if np.argmax(pad_axis.value) == 0:
        return  h_wrist[0] > 0 and h_wrist[0] < length and\
                h_wrist[2] > 0 and h_wrist[2] < width
    # if the occlussion/pad is in the z-y plane
    elif np.argmax(pad_axis.value) == np.argmin(pad_dim):
        return  h_wrist[0] > 0 and h_wrist[0] < length and\
                h_wrist[2] > 0 and h_wrist[2] < width
    # if the occlussion/pad is in the x-y plane
    else:
        return  h_wrist[0] > 0 and h_wrist[0] < height and\
                h_wrist[1] > 0 and h_wrist[1] < length

def get_joint_occlussion(joint_1,joint_2,pad_x_index,pad_y_index,pad_proj,pad_orth):
    occlussion_count = 0
    r_m, r_b= get_line([joint_1[pad_x_index],joint_1[pad_y_index]],
            [joint_2[pad_x_index],joint_2[pad_y_index]])
    h_line = lambda x: r_m*x + r_b
    x_points = [joint_1[pad_x_index],joint_2[pad_x_index]]
    for x in range(0,int(pad_proj)):
        for y in range(0, int(pad_orth)):
            if is_above_line([x,y], h_line) and\
                x >= min(x_points) and x <= max(x_points):
                occlussion_count += 1
                # Delete
                # draw_point = np.array([0,0,0], dtype=float)
                # draw_point[pad_x_index] = x
                # draw_point[pad_y_index] = y
                # draw_point += pad_origin
                # d_points.append(vec(*draw_point))
    # DELETE
    # if len(d_points)>0:
        # points(pos=d_points, radius=1, color=color.red)
    return occlussion_count

# Input: a DH simbolic row. Read the readme.md for more
# information about the function format
# output: a  homogeneous
# transformation for the frame of joint i-1 to the frame
# of the joint i. Each homogeneous transformation will be
# a lambda-function that will receive all the current
# joint values and evaluate the current transformation matrix
# accordingly.
def get_transformation(alpha, a, d, theta):
    r = pi/180 # constant to convert degrees to radians
    alpha_i = alpha*r
    theta_i = theta*r
    t = Matrix([
        [cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), a*cos(theta_i)],
        [sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), a*sin(theta_i)],
        [0, sin(alpha_i), cos(alpha_i), d],
        [0, 0, 0, 1],
        ])
    return t

# Input: a DH simbolic matrix. Read the readme.md for more
# information about the function format
# output: a list of homogeneous
# transformations from the frame of joint i-1 to the frame
# of the joint i. Each homogeneous transformation will be
# a lambda-function that will receive all the current
# joint values and evaluate the current transformation matrix
# accordingly.
def get_transformations(dh, variables):
    row, col = dh.shape
    t_matrix_list = []
    r = pi/180 # constant to convert degrees to radians
    for i in range(row):
        alpha_i = dh[i,0]*r
        a_i = dh[i,1]
        d_i = dh[i,2]
        theta_i = dh[i,3]*r
        t = Matrix([
            [cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), a_i*cos(theta_i)],
            [sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), a_i*sin(theta_i)],
            [0, sin(alpha_i), cos(alpha_i), d_i],
            [0, 0, 0, 1],
            ])
        t_matrix_list.append(lambdify(variables, t, "numpy"))
    return t_matrix_list

def diff_angle_base(p1,p2,pbase):
    v1 = p1 if isinstance(p1, vector) else vector(*p1)
    v2 = p2 if isinstance(p2, vector) else vector(*p2)
    base = pbase if isinstance(pbase, vector) else vector(*pbase)
    direction = v1.cross(v2)
    # if the direction is negative
    if diff_angle(base,direction) > pi/2:
       return -diff_angle(v1,v2)
    # if the direction is positive
    else:
       return diff_angle(v1,v2)

if __name__ == "__main__":
    # execute only if run as a script
    main()

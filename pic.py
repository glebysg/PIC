import numpy as np
from vpython import *
import copy
from pprint import pprint as pp
from helpers import *
import math
import cv2

class robotLink:
    def __init__(self,alpha, a, d, theta, min_angle, max_angle):
        self.alpha = alpha
        self.a = a
        self.d = d
        self.sym_joint = theta
        self.joint_val = 0
        self.min_angle  = min_angle
        self.max_angle = max_angle
        # the lambidified rotation matrix wrt the base
        self.rotation = None
        # set the lenght
        # we need to use the hypothenuse if we have a and d lenght
        if self.a != 0 and self.d !=0:
            self.length = math.sqrt(self.a**2+self.d**2)
        # if we have a d lenght
        elif d != 0:
            self.length = d
        # if we have an a lenght or the legth is zero
        else:
            self.length = a
        ######### visual components ############
        self.frame = None
        self.v_d = None
        self.v_a = None
        self.v_joint = None
        # small shpere to smoothen the apearance of
        # the joint when we have a and d distaces
        self.v_elbow = None

    # get the current matrix w.r.t the base
    def eval_rot(self,joint_vals):
        return self.rotation(*joint_vals)

class robotChain:
    # TODO initialize de DH matrix and sym joints in a more user friendly way
    # e.g. directly from a file
    def __init__(self, dh_params, joint_vars, ranges,
            base_matrix=None, tolerance=0.1, iterations=10,
            pose_imitation=False, human_joint_index=None, soften=0,
            filtering=True, filter_threshold=10, render_task=False,
            render_scale=1):
        # param constraint checkup
        if pose_imitation and human_joint_index is None:
            raise ValueError('the parameter human_joint_index must be \
                        specified when using pose imitation')
            raise ValueError('the parameter human_joint_index must be \
                   a list of three indices(integers) indicating\
                   shoulder, elbow and gripper')
        # symbolic dhlist of the chain
        self.dh_params = dh_params
        # symbolic names of the joints
        self.joint_vars = joint_vars
        # list of the robotic links
        self.rob_links = []
        # joint ranges
        self.ranges = ranges
        # current joint angles
        self.joint_vals = None
        # rotation matrix of the base
        if base_matrix is None:
            self.base_matrix = eye(4)
        else:
            self.base_matrix = base_matrix
        self.base = np.array(self.base_matrix).astype(np.float64)[0:3,3]
        self.base_rot = lambdify(self.joint_vars,self.base_matrix,'numpy')
        # error tolerance
        self.tolerance = tolerance
        self.iterations = iterations
        # for animation
        self.render_task = render_task
        self.render_scale = render_scale
        # pose imitation settings
        self.pose_imitation=pose_imitation
        self.human_joint_index = human_joint_index
        # Create an empty list for the constraints and for the joint points
        self.pose_constraints = []
        self.points = []
        self.conic_constraints = None
        # Filtering flag to smooth movement in pose imitation mode
        self.filtering = filtering
        # Angle change threshold at witch filtering occurs
        self.filter_threshold = filter_threshold
        # Base parameters for animating the constraints for the pose imitation
        if pose_imitation:
            self.base_lenght = 6
            self.base_offsets = self.get_base_offsets()
            self.soften = soften
        self.iter_counter = 0
        self.filtered_counter = 0

    def get_base_offsets(self):
        base_offsets = [[-1,1,-1],[1,1,-1],
                [1,1,1],[-1,1,1],[-1,-1,-1],
                [1,-1,-1],[1,-1,1],[-1,-1,1]]
        return np.array(base_offsets)

    def create_constraints(self, constraints):
        constraints = [c - 1 for c in constraints]
        self.pose_constraints = []
        # Shoulder
        self.pose_constraints.append(
                (self.human_joint_index[0],constraints[0],'out'))
        # Elbow
        self.pose_constraints.append(
                (self.human_joint_index[1],constraints[1],'in'))
        self.pose_constraints.append(
                (self.human_joint_index[1],constraints[2],'out'))
        # Hand
        self.pose_constraints.append(
                (self.human_joint_index[2],constraints[3],'in'))

    def create_conic_constraints(self, human_points):
        self.human_axis = []
        # Shoulder
        axis = normalize(human_points[1] - human_points[0])
        self.human_axis.append(vec(*axis))
        # Elbow
        axis = normalize(human_points[0] - human_points[1])
        self.human_axis.append(vec(*axis))
        axis = normalize(human_points[2] - human_points[1])
        self.human_axis.append(vec(*axis))
        # Hand
        axis = normalize(human_points[1] - human_points[2])
        self.human_axis.append(vec(*axis))

    def init_skeleton(self, init_angles, d_first, init_constraints=None, conic_constraints=None):
        # Check if init_constraints is present when using pose imitation
        if self.pose_imitation and conic_constraints is None and \
            ((init_constraints is None) or (len(init_constraints)==0)) :
            raise ValueError('the parameter init_constraints cannot \
                       be empty when using pose imitation')
        # Initialize the canvas
        # scene.width = 1200
        # scene.height = 800
        # Initialize the joints
        self.joint_vals = init_angles
        self.d_first = d_first
        # create the robot links
        # accumulated matrix multiplication
        accum_rotations = copy.copy(self.base_matrix)
        # last evaluated maxtix
        prev_eval  = np.array(self.base_matrix).astype(np.float64)
        # from the first joint to the gripper
        # The rotation at zero is the base origin rotation
        for i in range(len(self.joint_vals)):
            # Initialize a new link
            dh =  self.dh_params.row(i)
            min_angle = self.ranges[i,0]
            max_angle = self.ranges[i,1]
            m_rot = get_transformation(*dh)
            # Get the rotation w.r.t the base
            accum_rotations = accum_rotations*m_rot
            lambda_rot = lambdify(self.joint_vars, accum_rotations, "numpy")
            link = robotLink(*dh,min_angle,max_angle)
            link.rotation = lambda_rot
            ###############################################
            ###### create the IK elements to be drawn######
            ###############################################
            current_eval = link.eval_rot(init_angles)
            # draw the joint frames w.r.t the vpython frame
            link.frame = draw_reference_frame(prev_eval,transform=True)
            # if we are at the gripper
            if (i ==  len(init_angles) -1):
                self.gripper_frame = draw_reference_frame(current_eval,transform=True)
            # get the previous and current evalulated rotation
            # with respect to the vpython frame of reference
            current_eval_vpython = coppelia_frame_to_vpython(current_eval)*100
            prev_eval_vpython = coppelia_frame_to_vpython(prev_eval)*100
            # get link origin and legth
            pos = vec(*(prev_eval_vpython[0:3,3]))
            a_len = float(link.a*100)
            d_len = float(link.d*100)
            # draw the joint
            link.v_joint = sphere(pos=pos,color=color.orange, radius = 4)
            # if the the "d" leght comes first in the robot
            if self.d_first[i]:
                # draw d
                if d_len > 0:
                    # get the z-1 axis of  w.r.t the origin
                    d_orientation = vec(*prev_eval_vpython[0:3,2])
                    axis = norm(d_orientation)*float(d_len)
                    link.v_d = cylinder(pos=pos, axis=axis,
                            color=color.orange,radius=2)
                # draw a
                if a_len > 0:
                    # if we had d before
                    if d_len > 0:
                        # displace the position of the link
                        pos = pos + axis
                        # draw a small sphere to make the joint looks smoother
                        link.v_elbow =  sphere(pos=pos,color=color.orange, radius = 2)
                    # get the x axis of  w.r.t the origin
                    a_orientation = vec(*current_eval_vpython[0:3,0])
                    axis= norm(a_orientation)*float(a_len)
                    link.v_a = cylinder(pos=pos, axis=axis,
                            color=color.orange,radius=2)
            else:
                print("IMPLEMENT 'a' distance before 'd'")
                exit()
            self.rob_links.append(link)
            prev_eval = copy.copy(current_eval)
            ########### finish visual IK

        ###############################################
        ######   create the visual constraints   ######
        ###############################################
        # Create the initial pose
        if self.pose_imitation:
            self.create_constraints(init_constraints)
            self.graphic_constraints = []
            self.conic_constraints = conic_constraints
        # initialize the points
        self.points = self.get_points()
        ########### Create the visual constraints
        # if we are going to use conic constraints
        if self.conic_constraints:
            axis_len = self.base_lenght*2
            is_out = True
            for angle in conic_constraints:
                radius = np.tan(np.radians(angle/2.0))*axis_len
                cone_color = color.orange if is_out else color.yellow
                self.graphic_constraints.append(
                    cone(pos=vec(0,0,0),
                        axis=vec(axis_len,0,0),
                        radius=radius,
                        opacity=0.5,
                        color=cone_color))
                is_out = not is_out
        # if we are using cubic constraints
        else:
            prev_joint = -1
            for index in range(len(self.pose_constraints)):
                # Create the cubes representing the constraints
                constraint = self.pose_constraints[index]
                current_joint = constraint[0]
                if prev_joint != current_joint:
                    constraint_cluster = []
                    for i in range(len(self.base_offsets)):
                        c_color = color.orange if i == constraint[1] else color.white
                        constraint_cluster.append(box(pos=vec(0,0,0),
                                length=self.base_lenght, height=self.base_lenght, width=self.base_lenght,
                                opacity=0.5, color=color.white))
                    self.graphic_constraints.append(constraint_cluster)
                    prev_joint = current_joint
        ########### finish visual constraints
        # create task-based elements
        if self.render_task == "assembly":
            self.assembly_piece =  box(pos=vec(0,0,0),
                    size =vec(28,1,10)*self.render_scale,
                    texture=textures.metal)
        # Update constraint position
        if self.pose_imitation:
            if self.conic_constraints:
                self.update_conic_constraints
            else:
                self.update_constraints()

    def update_constraints(self):
        prev_joint = -1
        index_offset = 0
        # Reset all the cubes to be white
        for cube_array in self.graphic_constraints:
            for cube in cube_array:
                cube.color = color.white
        # Update the positions and colors of the cubes
        for constr_index in range(len(self.pose_constraints)):
            constraint = self.pose_constraints[constr_index]
            # Get the position of the joint for the costraint
            current_joint = constraint[0]
            # Get the constraint cube index
            constraint_cube_index = constraint[1]
            # Get the constraint type
            constraint_type = constraint[2]
            # update the index for the constraints only
            if prev_joint == current_joint:
                index_offset =+ 1
                c_color = color.orange if constraint_type == "out" else color.yellow
                self.graphic_constraints[constr_index-index_offset][constraint_cube_index].color =\
                       c_color
            # if the we have changed joints
            else:
                # Get the center of the cube cluster. The graphic_ik
                # Variable has the joints and the links, thus, we multiply
                # by 2 to get to the correct joint.

                # If we are at the last joint
                if current_joint*2 == len(self.graphic_ik):
                    center_pos = self.gripper.pos
                else:
                    center_pos = self.graphic_ik[current_joint*2].pos
                # Update the positions of each joint arround the center
                for base_index in range(len(self.base_offsets)):
                    x_off, y_off, z_off = self.base_offsets[base_index]
                    offset = vec(self.base_lenght*x_off, self.base_lenght*y_off, self.base_lenght*z_off)
                    self.graphic_constraints[constr_index - index_offset][base_index].pos = \
                            pos=center_pos + offset
                    # if we need to update the cube color
                    if base_index == constraint_cube_index:
                        c_color = color.orange if constraint_type == "out" else color.yellow
                        self.graphic_constraints[constr_index - index_offset][base_index].color = c_color
            prev_joint = current_joint

    def update_conic_constraints(self):
        for index in range(len(self.human_axis)):
            # get the constraint that corresponds to the human joint
            constraint = self.pose_constraints[index]
            # Get the position of the joint for the costraint
            current_joint = constraint[0]
            cone_len = mag(self.graphic_constraints[index].axis)
            cone_axis = copy.copy(self.human_axis[index])
            cone_axis = norm(cone_axis)*cone_len
            if current_joint*2 == len(self.graphic_ik):
                cone_pos = self.gripper.pos
            else:
                cone_pos = self.graphic_ik[current_joint*2].pos
            cone_pos += cone_axis
            # Get it in the opposite direction to draw the cone correctly
            # cone_axis = -1*cone_axis
            # update axis and position
            self.graphic_constraints[index].axis = -cone_axis
            self.graphic_constraints[index].pos = cone_pos

    def draw_chain(self):
        # last evaluated maxtix
        prev_eval  = np.array(self.base_matrix).astype(np.float64)
        arrow_size = 10
        for i in range(len(self.joint_vals)):
            link = self.rob_links[i]
            current_eval = link.eval_rot(self.joint_vals)
            # Reorient and move the reference Frame
            frame_pos = vector(*((coppelia_pt_to_vpython(prev_eval[:,3])*100)))
            v_frame = coppelia_frame_to_vpython(prev_eval)
            link.frame[0].pos = frame_pos
            link.frame[1].pos = frame_pos
            link.frame[2].pos = frame_pos
            link.frame[0].axis = norm(vector(*v_frame[0:3,0]))*arrow_size
            link.frame[1].axis = norm(vector(*v_frame[0:3,1]))*arrow_size
            link.frame[2].axis = norm(vector(*v_frame[0:3,2]))*arrow_size
            # reorient and move the links
            link.v_joint.pos = frame_pos
            if (i ==  len(self.joint_vals) -1):
                gripper_pos = vector(*((coppelia_pt_to_vpython(prev_eval[:,3])*100)))
                gripper_frame = coppelia_frame_to_vpython(prev_eval)
                self.gripper_frame[0].pos = gripper_pos
                self.gripper_frame[1].pos = gripper_pos
                self.gripper_frame[2].pos = gripper_pos
                self.gripper_frame[0].axis = norm(vector(*gripper_frame[0:3,0]))*arrow_size
                self.gripper_frame[1].axis = norm(vector(*gripper_frame[0:3,1]))*arrow_size
                self.gripper_frame[2].axis = norm(vector(*gripper_frame[0:3,2]))*arrow_size
            # if the the "d" leght comes first in the robot
            if self.d_first[i]:
                # update d
                if link.d > 0:
                    # get the z-1 axis of  w.r.t the origin
                    d_orientation = vec(*v_frame[0:3,2])
                    axis = norm(d_orientation)*float(link.d*100)
                    link.v_d.pos = frame_pos
                    link.v_d.axis = axis
                # update a
                if link.a > 0:
                    # if we had d before
                    if link.d > 0:
                        # displace the position of the link
                        frame_pos = frame_pos + axis
                        link.v_elbow.pos = frame_pos
                    # get the x axis of  w.r.t the origin
                    v_frame_current = coppelia_frame_to_vpython(current_eval)
                    a_orientation = vec(*v_frame_current[0:3,0])
                    axis= norm(a_orientation)*float(link.a*100)
                    link.v_a.pos = frame_pos
                    link.v_a.axis = axis
            else:
                print("IMPLEMENT 'a' distance before 'd'")
                exit()
            prev_eval = copy.copy(current_eval)

        if self.pose_imitation:
            if self.conic_constraints:
                self.update_conic_constraints()
            else:
                self.update_constraints()
        # Render Task-dependent elements in the chain
        if self.render_task == "assembly":
            axis = vec(*normalize(axis.value))
            self.assembly_piece.pos=vec(*self.points[-1])+axis*self.assembly_piece.length/2
            # self.assembly_piece.pos=vec(*self.points[-1])+axis*math.sqrt(20)
            self.assembly_piece.axis=axis*self.assembly_piece.length

    def get_gripper_pos(self):
        last_rot = self.rob_links[1].eval_rot(self.joint_vals)
        return vector(*last_rot[0:3,3])

    def get_points(self):
        points = [self.base]
        for link in self.rob_links:
            current_rot = link.eval_rot(self.joint_vals)
            point = current_rot[0:3,3]
            points.append(point)
        return np.array(points)

    def forward(self):
        backward_points = self.backward_points[::-1]
        prev_frame = np.array(self.base_matrix).astype(np.float64)
        #delete
        copp_points = [prev_frame[:,3]]
        for i in range(len(self.rob_links)):
            # Get the joint pos evaluated at the min, max, and zero
            link = self.rob_links[i]
            vals = copy.copy(self.joint_vals)
            if link.length == 0 and i < (len(self.rob_links)-1):
                next_link = self.rob_links[i+1]
                vals[i] = link.min_angle
                joint_min = next_link.eval_rot(vals)
                vals[i] =  0
                joint_zero = next_link.eval_rot(vals)
                vals[i] = link.max_angle
                joint_max = next_link.eval_rot(vals)
            elif link.length == 0 and i == (len(self.rob_links)-1):
                print("implement the case of gripper with no length")
                exit()
            else:
                vals[i] = link.min_angle
                joint_min = link.eval_rot(vals)
                vals[i] =  0
                joint_zero = link.eval_rot(vals)
                vals[i] = link.max_angle
                joint_max = link.eval_rot(vals)
            # project the target to the plane created
            # by the range of motion of the joint
            # delete
            #######################
            # Draw the min,zero,max angles as a curve
            # if i==6:
                # vrep_min = vector(*(coppelia_pt_to_vpython(joint_min[:,3])*100).astype('float64'))
                # vrep_zero = vector(*(coppelia_pt_to_vpython(joint_zero[:,3])*100).astype('float64'))
                # vrep_max = vector(*(coppelia_pt_to_vpython(joint_max[:,3])*100).astype('float64'))
                # sphere(pos=vrep_min, color = color.white,radius=4)
                # sphere(pos=vrep_zero, color = color.cyan,radius=4)
                # sphere(pos=vrep_max, color = color.blue,radius=4)
            # finish drawing
            #######################
            p_target = pt_project_to_plane(joint_min[0:3,3], joint_zero[0:3,3], joint_max[0:3,3], backward_points[i+1])
            # if the points of the plane are colinear
            # just choose the angle value at zero and go to the next joint
            if np.sum(p_target) == 0:
                self.joint_vals[i] = 0
                continue
            # add the d distance to the rotation frame
            rot_frame = prev_frame[0:3,3] + np.array([0,0,self.rob_links[i].d]).astype('float64')
            # delete
            # if i==6:
                # vrep_frame  = vector(*(coppelia_pt_to_vpython(np.append(p_target,1))*100).astype('float64'))
                # sphere(pos=vrep_frame, color = color.magenta,radius=4)
            p_zero = joint_zero[0:3,3]- rot_frame
            p_target = p_target - rot_frame
            # get the angle between the neutral joint pos (joint val=0) and the current pos
            diff_angle = math.degrees(diff_angle_base(p_zero, p_target, prev_frame[0:3,2]))
            # if is greater than the max, make the max
            if (diff_angle > link.max_angle):
                diff_angle = link.max_angle
            # if is smaller than the min, make the min.
            elif (diff_angle < link.min_angle):
                diff_angle = link.min_angle
            # update the joint values
            self.joint_vals[i] = diff_angle
            # evaluate the rotation
            prev_frame = link.eval_rot(self.joint_vals)
            #delete
            # copp_points.append(prev_frame[:,3])
            # if i == 5:
                # coppelia_fw = [coppelia_pt_to_vpython(p)*100 for p in copp_points]
                # draw_debug(coppelia_fw,color.orange, opacity=0.5)
                # exit()
        #delete
        # coppelia_fw = [coppelia_pt_to_vpython(p)*100 for p in copp_points]
        # draw_debug(coppelia_fw,color.orange, opacity=0.5)
        # exit()

    def backward(self):
        target_point = self.target.copy()
        backward_points = [target_point]
        for i in range(len(self.points)-2,-1,-1):
            new_orientation = normalize(self.points[i] - target_point)
            backward_point = target_point + \
                    (new_orientation*self.rob_links[i].length).astype('float64')
            backward_points.append(backward_point)
            target_point = backward_point
            if i%2 ==1:
                sphere(pos=vec(*(coppelia_pt_to_vpython(backward_point)*100)), color=color.blue, radius=5)
            else:
                sphere(pos=vec(*(coppelia_pt_to_vpython(backward_point)*100)), color=color.green, radius=5)
            sleep(2)
        self.backward_points = backward_points
        # delete
        # coppelia_bw_points = [coppelia_pt_to_vpython(np.append(p,1))*100 for p in backward_points]
        # draw_debug(coppelia_bw_points,color.blue, opacity=0.5)

    def pic_forward(self):
        backward_points = self.backward_points[::-1]
        # tuples of joint index where the constraint is, type of comnstraint
        human_joints = [(constr[0], constr[2]) for constr in self.pose_constraints]
        for i in range(len(self.points)-1):
            # reorient towards the backward chain
            target = backward_points[i+1]
            new_orientation = normalize(target-self.points[i])
            # Orient towards the constraints if we are in pose
            # imitation mode
            # if the constraint is going out of the cube
            if self.pose_imitation and (i,'out') in human_joints:
                constr_index = human_joints.index((i,'out'))
                constr_region = self.pose_constraints[constr_index][1]
                ##### check if the link intercepts the constraint region
                # check the intersection by conic constraint
                if self.conic_constraints is not None:
                    axis = self.human_axis[constr_index]
                    constraint_angle = self.conic_constraints[constr_index]
                    intersects = is_conic_intersection(
                           self.points[i],
                            target,
                            axis,
                            constraint_angle
                            )
                   # if it doesnt intersect project the link so it falls
                    # inside the conic constraint
                    if not intersects:
                        # Change the orientation to the one of the projection
                        new_orientation = get_conic_projection(self.points[i],
                                target,
                                axis,
                                constraint_angle)
                        new_orientation = normalize(new_orientation)
                # check the intersection by cuadrant constraint
                else:
                    intersects = is_constraint_intersection(
                            self.points[i],
                            self.base_offsets,
                            constr_region,
                            target)
                    # if it doesnt intersect, find the the side of the
                    # sub-cube that the link can be projected to.
                    if not intersects:
                        # Change the orientation to the one of the projection
                        new_orientation = get_projection(self.points[i],
                                self.base_offsets,
                                constr_region,
                                target,
                                self.soften)
                        new_orientation = normalize(new_orientation)
            # change the position of the point at the
            # end of the link
            self.chain[i].orientation = new_orientation
            self.points[i+1] = self.points[i] + new_orientation*self.chain[i].length

    def pic_backward(self):
        target = self.target.copy()
        backward_points = [target]
        # if the constraint is going out of the cube
        human_joints = [(constr[0], constr[2]) for constr in self.pose_constraints]
        for i in range(len(self.points)-2,-1,-1):
            new_orientation = normalize(self.points[i] - target)
            # Orient towards the constraints if we are in pose
            # imitation mode
            # if there are constraints for the target
            # and the constraint is going into the cube
            if self.pose_imitation and (i+1, 'in') in human_joints:
                constr_index = human_joints.index(((i+1),'in'))
                constr_region = self.pose_constraints[constr_index][1]
                ##### check if the link intercepts the constraint region
                # check the intersection by conic constraint
                if self.conic_constraints is not None:
                    axis = self.human_axis[constr_index]
                    constraint_angle = self.conic_constraints[constr_index]
                    intersects = is_conic_intersection(
                            target,
                            self.points[i],
                            axis,
                            constraint_angle)
                    # if it doesnt intersect project the link so it falls
                    # inside the conic constraint
                    if not intersects:
                        # Change the orientation to the one of the projection
                        new_orientation = get_conic_projection(
                                target,
                                self.points[i],
                                axis,
                                constraint_angle)
                        new_orientation = normalize(new_orientation)
                # check the intersection by cuadrant constraint
                # check if the link intercepts the constraint region
                # since we are going backwards, the constraint region
                # is centered at the target, and the target is the
                # previous joint (self.point[i])
                else:
                    intersects = is_constraint_intersection(
                            target,
                            self.base_offsets,
                            constr_region,
                            self.points[i])
                    # if it doesnt, find the the side of the sub-cube that
                    # the link can be projected to.
                    if not intersects:
                        # Change the orientation to the one of the projection
                        new_orientation = get_projection(
                                target,
                                self.base_offsets,
                                constr_region,
                                self.points[i],
                                self.soften)
                        new_orientation = normalize(new_orientation)
            backward_point = target + new_orientation*self.chain[i].length
            backward_points.append(backward_point)
            target = backward_point
        self.backward_points = backward_points

    def correct_joints(self, error, beta=1):
        for i in range(len(self.rob_links)):
            # self.joint_vals[i] = diff_angle
            link = self.rob_links[i]
            vals = copy.copy(self.joint_vals)
            val_plus = vals[i]+beta if (link.max_angle < vals[i]+beta) else link.max_angle
            val_minus = vals[i]-beta if (link.min_angle > vals[i]-beta) else link.min_angle
            vals[i] = val_plus
            error_max = distance(self.rob_links[-1].eval_rot(vals)[0:3,3],self.target)
            vals[i] = val_minus
            error_min = distance(self.rob_links[-1].eval_rot(vals)[0:3,3],self.target)
            if error_min < error and error_min < error_max:
                print("replaced error min on joint", i)
                self.joint_vals[i] = val_minus
            elif error_max < error and error_max < error_min:
                print("replaced error max on joint", i)
                self.joint_vals[i] = val_plus

    def solve(self, target, constraints=None, humman_points=None, lock_threshold=0.0001):
        self.iter_counter += 1
        # Initialize constraints
        if self.pose_imitation and self.conic_constraints is not None\
            and humman_points is not None:
            self.create_conic_constraints(humman_points)
        if self.pose_imitation and constraints is not None:
            self.create_constraints(constraints)
        # Check if the point is reachable
        self.target = np.array(target, dtype=float)
        distance_to_target = np.linalg.norm(self.target-self.base)
        if (sum([l.length for l in self.rob_links]) > distance_to_target):
            # initialize the points
            self.points = self.get_points()
            if self.filtering:
                prev_points = copy.deepcopy(self.points)
            # get distance between target and goal
            error = distance(self.points[-1],self.target)
            count = 0
            prev_error = -10000000000000
            while error > self.tolerance:
                if self.pose_imitation:
                    self.pic_backward()
                    self.pic_forward()
                else:
                    self.backward()
                    self.forward()
                error = distance(self.points[-1],self.target)
                if count > self.iterations:
                        break
                # if the robot is locked
                # if abs(prev_error-error)<lock_threshold:
                    # self.correct_joints(error, beta=1)
                prev_error = error
                self.points = self.get_points()
                count += 1
            # if we are in pose imitation mode and we
            # are filtering the data
            needs_filter = False
            if self.pose_imitation and self.filtering:
                # check if any angle in the chain goes over a threshhold
                prev_link = self.chain[0]
                for index in range(len(self.points)-2):
                    v1 = vec(*self.points[index]) -  vec(*self.points[index + 1])
                    prev_v1 = vec(*prev_points[index]) -  vec(*prev_points[index + 1])
                    v2 = vec(*self.points[index + 2]) -  vec(*self.points[index + 1])
                    prev_v2 = vec(*prev_points[index + 2]) -  vec(*prev_points[index + 1])
                    angle = degrees(diff_angle(v1,v2))
                    prev_angle = degrees(diff_angle(prev_v1,prev_v2))
                    if abs(angle-prev_angle) > self.filter_threshold:
                        needs_filter = True
                        break
            if needs_filter:
                self.filtered_counter += 1
                self.points = prev_points
                error = distance(self.points[-1],self.target)
                count = 0
                while error > self.tolerance:
                    self.backward()
                    self.forward()
                    error = distance(self.points[-1],self.target)
                    if count > self.iterations:
                            break
                    count += 1

            self.draw_chain()
        else:
            print("Not reachable")
    def animate(self):
        rate(100)
        # if the mouse is being dragged
        if self.drag:
            # Animate the solved ik chain
            self.draw_chain()


# TODO change to the appropriate function
# Helper function for reading arms
# create ik chain from file
# the file must be in the following format:
# left-link-len, left-x-orientation, left-y-orientation,
# left-z-orientation, right-link-len...same for the right arm
# each line is a link
# returns two list of links, one for each arm
def read_arm(path):
    left_chain = []
    right_chain = []
    links = np.loadtxt(path, delimiter=",")
    for link in links:
        left_chain.append(ikLink(length=link[0],orientation=link[1:4]))
        right_chain.append(ikLink(length=link[4],orientation=link[5:8]))
    return left_chain, right_chain



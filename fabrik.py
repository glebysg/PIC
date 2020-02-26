import numpy as np
from vpython import *
import copy
from pprint import pprint as pp
from helpers import *
import math

class ikLink:
    def __init__(self, length=1, orientation=[1,0,0]):
        self.length = length
        self.orientation = np.array(orientation, dtype=float)

class ikChain:
    def __init__(self,base=[0,0,0],chain=[], tolerance=0.1, iterations=10,
            pose_imitation=False, human_joint_index=None, soften=0,
            filtering=True, filter_threshold=10, render_task=False,
            render_scale=1):
        # initialize the canvas
        # param constraint checkup
        if pose_imitation and human_joint_index is None:
            raise ValueError('the parameter human_joint_index must be \
                        specified when using pose imitation')
        elif pose_imitation and len(human_joint_index)!=3:
            raise ValueError('the parameter human_joint_index must be \
                   a list of three indices(integers) indicating\
                   shoulder, elbow and gripper')
        # a 3d point indicating the base of the kinematic chain
        self.base = np.array(base, dtype=float)
        # a list of iklinks
        self.chain = chain
        # error tolerance
        self.tolerance = tolerance
        self.iterations = iterations
        # for animation
        self.drag = True
        self.render_task = render_task
        self.render_scale = render_scale
        # self.ik_sphere=None
        self.ik_sphere = sphere(pos=vec(0,0,0), color=color.red, radius=3)
        self.ik_sphere.visible = False
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

    def init_skeleton(self, init_constraints=None, conic_constraints=None):
        # Check if init_constraints is present when using pose imitation
        if self.pose_imitation and conic_constraints is None and \
            ((init_constraints is None) or (len(init_constraints)==0)) :
            raise ValueError('the parameter init_constraints cannot \
                       be empty when using pose imitation')
        # Create the initial pose
        if self.pose_imitation:
            self.create_constraints(init_constraints)
            self.graphic_constraints = []
            self.conic_constraints = conic_constraints
        # initialize the points
        self.points = self.get_points()
        # draw a box on the base
        self.base_box = box(pos=vec(*self.base), length=10, height=3, width=10)
        # Draw each element in the ik chain
        scene.width = 1200
        scene.height = 800
        self.graphic_ik = []
        axis = None
        for index in range(len(self.chain)):
            # Normalize the orientation of the ik link
            pos = vec(*self.points[index])
            axis = vec(*(normalize(self.chain[index].orientation)*self.chain[index].length))
            joint =  sphere(pos=pos,color=color.green, radius = 4)
            link = cylinder(pos=pos, axis=axis, radius=2)
            self.graphic_ik.append(joint)
            self.graphic_ik.append(link)
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
        # add gripper
        self.gripper = pyramid(pos=vec(*self.points[-1]), size=vec(2,4,4),
                axis=axis, color=color.green)
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
        # Create the ik ball to manipulate the chain and bind the drag
        self.ik_sphere.pos=vec(*self.points[-1])
        self.animation_pos=copy.copy(self.points[-1])
        def down():
            self.drag = True
        def move():
            if self.drag: # mouse button is down
                self.ik_sphere.color = color.cyan
                self.ik_sphere.pos = scene.mouse.pos
                self.new_pos = vec_to_np(self.ik_sphere.pos, float)
                if not np.array_equal(self.animation_pos,self.new_pos):
                    self.animation_pos = copy.copy(self.new_pos)
                    #Get the target from the sphere
                    self.solve(self.animation_pos)
        def up():
            self.ik_sphere.color = color.red
            self.drag = False
        scene.bind("mousedown", down)
        scene.bind("mousemove", move)
        scene.bind("mouseup", up)

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
        axis = None
        for index in range(len(self.chain)):
            # Normalize the orientation of the ik link
            pos = vec(*self.points[index])
            axis = vec(*(normalize(self.chain[index].orientation)\
                    *self.chain[index].length))
            self.graphic_ik[index*2].pos = pos
            self.graphic_ik[index*2+1].pos = pos
            self.graphic_ik[index*2+1].axis = axis
        self.gripper.pos = vec(*self.points[-1])
        self.gripper.axis = axis
        if self.pose_imitation:
            if self.conic_constraints:
                self.update_conic_constraints()
            else:
                self.update_constraints()
        # Render Task-dependent elements in the chain
        if self.render_task == "assembly":
            axis = vec(*normalize(axis.value))
            self.gripper.size=vec(6,4,4)
            self.assembly_piece.pos=vec(*self.points[-1])+axis*self.assembly_piece.length/2
            # self.assembly_piece.pos=vec(*self.points[-1])+axis*math.sqrt(20)
            self.assembly_piece.axis=axis*self.assembly_piece.length
        else:
            self.gripper.size=vec(2,4,4)

    def get_points(self):
        points = [self.base]
        previous_point = self.base
        for i in range(len(self.chain)):
            point = previous_point +\
                normalize(self.chain[i].orientation)*self.chain[i].length
            points.append(point)
            previous_point = point
        return points

    def forward(self):
        backward_points = self.backward_points[::-1]
        for i in range(len(self.points)-1):
            # reorient towards the backward chain
            new_orientation = normalize(backward_points[i+1]-self.points[i])
            self.chain[i].orientation = new_orientation
            # change the position of the point at the
            # end of the link
            self.points[i+1] = self.points[i] + new_orientation*self.chain[i].length

    def backward(self):
        target_point = self.target.copy()
        backward_points = [target_point]
        for i in range(len(self.points)-2,-1,-1):
            new_orientation = normalize(self.points[i] - target_point)
            backward_point = target_point + new_orientation*self.chain[i].length
            backward_points.append(backward_point)
            target_point = backward_point
        self.backward_points = backward_points

    def py_forward(self):
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

    def py_backward(self):
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

    def solve(self, target, constraints=None, humman_points=None):
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
        # if the target is not reachable
        if (sum([l.length for l in self.chain]) < distance_to_target)\
           and not self.pose_imitation:
            # Get goal orientation
            goal_orientation = self.target-self.base
            for i in range(len(self.chain)):
                self.chain[i].orientation = goal_orientation
            self.points = self.get_points()
        else:
            # initialize the points
            self.points = self.get_points()
            if self.filtering:
                prev_points = copy.deepcopy(self.points)
            # get distance between target and goal
            error = distance(self.points[-1],self.target)
            count = 0
            while error > self.tolerance:
                if self.pose_imitation:
                    # self.update_conic_constraints()
                    self.py_backward()
                    self.py_forward()
                    # exit(0)
                else:
                    self.backward()
                    self.forward()
                error = distance(self.points[-1],self.target)
                if count > self.iterations:
                        break
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
        self.gripper.pos = vec(*self.points[-1])
        self.draw_chain()

    def animate(self):
        rate(100)
        # if the mouse is being dragged
        if self.drag:
            # Animate the solved ik chain
            self.draw_chain()

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

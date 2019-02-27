import numpy as np
from utils import normalize, distance, vec_to_np
from vpython import *
import copy
from pprint import pprint as pp

class ikLink:
    def __init__(self, length=1, orientation=[1,0,0]):
        self.length = length
        self.orientation = np.array(orientation, dtype=float)

class ikChain:
    def __init__(self,base=[0,0,0],chain=[], tolerance=0.1, iterations=10,
            pose_imitation=False, human_joint_index=None):
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
        # self.ik_sphere=None
        self.ik_sphere = sphere(pos=vec(0,0,0), color=color.red, radius=3)
        self.pose_imitation=pose_imitation
        self.human_joint_index = human_joint_index
        # Create an empty list for the constraints
        self.pose_constraints = []
        # Base parameters for animating the constraints for the pose imitation
        if pose_imitation:
            self.base_lenght = 6
            self.base_offsets = [[-1,1,-1],[1,1,-1],
                    [1,1,1],[-1,1,1],[-1,-1,-1],
                    [1,-1,-1],[1,-1,1],[-1,-1,1]]

    def create_constraints(self, constraints):
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

    def init_skeleton(self, init_constraints=None):
        # Check if init_constraints is present when using pose imitation
        if self.pose_imitation and \
            (init_constraints is None) or (len(init_constraints)==0) :
            raise ValueError('the parameter init_constraints cannot \
                       be empty when using pose imitation')
        # Create the initial pose
        if self.pose_imitation:
            self.create_constraints(init_constraints)
            self.graphic_constraints = []
        # initialize the points
        self.points = self.get_points()
        # draw a box on the base
        self.base_box = box(pos=vec(*self.base), length=10, height=3, width=10)
        # Draw each element in the ik chain
        scene.width = 700
        scene.height = 700
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
        # Create the visual constraints
        prev_joint = -1
        for index in range(len(self.pose_constraints)):
            # Create the cubes representing the constraints
            constraint = self.pose_constraints[index]
            current_joint = constraint[0]
            if prev_joint == current_joint:
                # Get the last constraint and change the color to
                # reflect the current 'out' constraint
                self.graphic_constraints[-1][constraint[1]].color = color.orange
            else:
                constraint_cluster = []
                for i in range(len(self.base_offsets)):
                    c_color = color.orange if i == constraint[1] else color.white
                    constraint_cluster.append(box(pos=vec(0,0,0),
                            length=self.base_lenght, height=self.base_lenght, width=self.base_lenght,
                            opacity=0.5, color=c_color))
                self.graphic_constraints.append(constraint_cluster)
                prev_joint = current_joint
        # Update constraint position
        self.gripper = pyramid(pos=vec(*self.points[-1]), size=vec(2,4,4),
                axis=axis, color=color.green)
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
        print("pose constraints: ", self.pose_constraints)
        prev_joint = -1
        index_offset = 0
        # Reset all the cubes to be white
        for cube_array in self.graphic_constraints:
            for cube in cube_array:
                cube.color = color.white
        # Update the positions and colors of the cubes
        for constraint_index in range(len(self.pose_constraints)):
            constraint = self.pose_constraints[constraint_index]
            # Get the position of the joint for the costraint
            current_joint = constraint[0]
            # Get the constraint cube index
            constraint_cube_index = constraint[1]-1
            # Get the constraint type
            constraint_type = constraint[2]
            # update the index for the constraints only
            if prev_joint == current_joint:
                index_offset =+ 1
                c_color = color.orange if constraint_type == "out" else color.yellow
                self.graphic_constraints[constraint_index-index_offset][constraint_cube_index].color =\
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
                    self.graphic_constraints[constraint_index - index_offset][base_index].pos = \
                            pos=center_pos + offset
                    # if we need to update the cube color
                    if base_index == constraint_cube_index:
                        c_color = color.orange if constraint_type == "out" else color.yellow
                        self.graphic_constraints[constraint_index - index_offset][base_index].color = c_color
            prev_joint = current_joint
        pp([c.pos for c in np.array(self.graphic_constraints).flatten()])

    def draw_debug(self,points,color):
        axis = None
        self.ik_sphere.pos = vec(*self.target)
        self.ik_sphere.radius = 5
        for index in range(len(points)-1):
            # Normalize the orientation o f the ik link
            pos = vec(*points[index])
            length = distance(points[index],points[index+1])
            orientation = normalize(points[index+1]-points[index])
            axis = vec(*(orientation*length))
            joint =  sphere(pos=pos,color=color, radius = 4)
            link = cylinder(pos=pos, axis=axis, color=color,radius=2)

    def draw_chain(self):
        axis = None
        for index in range(len(self.chain)):
            # Normalize the orientation o f the ik link
            pos = vec(*self.points[index])
            axis = vec(*(normalize(self.chain[index].orientation)\
                    *self.chain[index].length))
            self.graphic_ik[index*2].pos = pos
            self.graphic_ik[index*2+1].pos = pos
            self.graphic_ik[index*2+1].axis = axis
        self.gripper.pos = vec(*self.points[-1])
        self.gripper.axis = axis
        self.gripper.size = vec(2,4,4)

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
        for i in range(len(self.chain)):
            # reorient towards the backward chain
            new_orientation = normalize(backward_points[i+1]-self.points[i])
            self.chain[i].orientation = new_orientation
            # change the position of the point at the
            # end of the link
            self.points[i+1] = self.points[i] + new_orientation*self.chain[i].length

    def backward(self):
        target_point = self.target
        self.backward_points = [target_point]
        for i in range(len(self.points)-2,-1,-1):
            to_target = normalize(self.points[i] - target_point)
            backward_point = target_point + to_target*self.chain[i].length
            self.backward_points.append(backward_point)
            target_point = backward_point

    def py_forward(self):
        backward_points = self.backward_points[::-1]
        for i in range(len(self.chain)):
            # reorient towards the backward chain
            new_orientation = normalize(backward_points[i+1]-self.points[i])
            self.chain[i].orientation = new_orientation
            # Orient towards the constraints if we are in pose
            # imitation mode
            human_links = [link[0] for link in self.pose_constraints]
            if self.pose_imitation and i in human_links:
                constraint_index = human_links.index(i)
                # if the constraint is going into the cube
                if self.pose_constraints[i][2] == 'out':
                    # check if the link intercepts the constraint region
                    # if it does, there is nothing to do

                    # if it doesnt, find the the side of the sub-cube that
                    # the link can be projected to.
                    pass
                # if the constraint is going out of the cube
                else:
                    pass

            # change the position of the point at the
            # end of the link
            self.points[i+1] = self.points[i] + new_orientation*self.chain[i].length

    def solve(self, target, constraints=None):
        # Initialize constraints
        if self.pose_imitation and constraints is not None:
            self.create_constraints(constraints)
        # Check if the point is reachable
        self.target = np.array(target, dtype=float)
        distance_to_target = np.linalg.norm(self.target-self.base)
        # if the target is not reachable
        if (sum([l.length for l in self.chain]) < distance_to_target):
            # Get goal orientation
            goal_orientation = self.target-self.base
            for i in range(len(self.chain)):
                self.chain[i].orientation = goal_orientation
            self.points = self.get_points()
        # if the target is reachable
        else:
            # initialize the points
            self.points = self.get_points()
            # get distance between target and goal
            error = distance(self.points[-1],self.target)
            count = 0
            while error > self.tolerance:
                self.backward()
                self.forward()
                self.draw_chain()
                error = distance(self.points[-1],self.target)
                if count > self.iterations:
                        break
                count += 1

    def animate(self):
        rate(100)
        # if the mouse is being dragged
        if self.drag:
            # Animate the solved ik chain
            self.draw_chain()


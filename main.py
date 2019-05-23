from vpython import *
from helpers import draw_reference_frame
from fabrik import ikLink, ikChain

def create_chain():
    link_chain = []
    human_joint_index = []
    # link_chain.append(ikLink(length=50,orientation=[1,0,0]))
    # link_chain.append(ikLink(length=30,orientation=[0.5,0.5,0]))
    # link_chain.append(ikLink(length=20,orientation=[0,0,1]))
    # link_chain.append(ikLink(length=70,orientation=[-0.5,-0.5,1]))
    # Simplified baxter
    link_chain.append(ikLink(length=6.9,orientation=[-1,0,0]))
    link_chain.append(ikLink(length=36.435,orientation=[-1,0,0]))
    link_chain.append(ikLink(length=37.429,orientation=[-1,0,0]))
    link_chain.append(ikLink(length=36.830,orientation=[-1,0,0]))
    human_joint_index = [0,3,4]
    init_constraints = [8,3,7,6]
    return link_chain, human_joint_index, init_constraints

def main():
    # draw x,y,z
    draw_reference_frame(-100,0,100,arrow_size=10)
    chain, human_joint_index, init_constraints = create_chain()
    chain = ikChain(chain=chain, pose_imitation=True,
            human_joint_index=human_joint_index,iterations=20)
    chain.init_skeleton(init_constraints=init_constraints)
    chain.solve([-10, -70.0, 15.0],init_constraints)
    # chain.solve([-83.8738,34.6046, -2.1450], init_constraints)
    # chain.animate()
    # while True:
        # chain.animate()

if __name__ == "__main__":
    main()

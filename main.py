from vpython import *
from fabrik import ikLink, ikChain

def create_chain():
    link_chain = []
    human_joint_index = []
    # link_chain.append(ikLink(length=50,orientation=[1,0,0]))
    # link_chain.append(ikLink(length=30,orientation=[0.5,0.5,0]))
    # link_chain.append(ikLink(length=20,orientation=[0,0,1]))
    # link_chain.append(ikLink(length=70,orientation=[-0.5,-0.5,1]))
    link_chain.append(ikLink(length=50,orientation=[1,0,0]))
    link_chain.append(ikLink(length=30,orientation=[0.5,0.5,0]))
    link_chain.append(ikLink(length=60,orientation=[0,0,1]))
    link_chain.append(ikLink(length=20,orientation=[-0.5,-0.5,1]))
    human_joint_index = [0,2,4]
    init_constraints = [7,4,3,8]
    return link_chain, human_joint_index, init_constraints

def main():
    chain, human_joint_index, init_constraints = create_chain()
    chain = ikChain(chain=chain, pose_imitation=True,
            human_joint_index=human_joint_index)
    chain.init_skeleton(init_constraints=init_constraints)
    # chain.animate()
    # while True:
        # chain.animate()

if __name__ == "__main__":
    main()

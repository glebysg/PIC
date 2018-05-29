from vpython import *
from fabrik import ikLink, ikChain

def create_chain():
    link_chain = []
    link_chain.append(ikLink(length=50,orientation=[1,0,0]))
    link_chain.append(ikLink(length=30,orientation=[0.5,0.5,0]))
    link_chain.append(ikLink(length=20,orientation=[0,0,1]))
    link_chain.append(ikLink(length=70,orientation=[-0.5,-0.5,1]))
    return link_chain

def main():
    chain = ikChain(chain=create_chain())
    chain.init_skeleton()
    # chain.animate()
    # while True:
        # chain.animate()

if __name__ == "__main__":
    main()

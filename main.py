from fabrik import ikLink, ikChain

def create_chain():
    link_chain = []
    link_chain.append(ikLink(length=50,orientation=[1,0,0]))
    link_chain.append(ikLink(length=50,orientation=[0.5,0.5,0]))
    link_chain.append(ikLink(length=20,orientation=[0,0,1]))
    return link_chain

def main():
    chain = ikChain(chain=create_chain())
    chain.init_skeleton()
    while True:
      chain.animate()

if __name__ == "__main__":
    main()

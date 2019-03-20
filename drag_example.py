from vpython import *
box()

drag = False
s = None # declare s to be used below

def down():
    global drag
    global s
    s = sphere(pos=scene.mouse.pos,
        color=color.red,
        size=0.2*vec(1,1,1))
    drag = True

def move():
    global drag
    global s
    if drag: # mouse button is down
        s.pos = scene.mouse.pos

def up():
    global drag
    global s
    s.color = color.cyan
    drag = False

scene.bind("mousedown", down)

scene.bind("mousemove", move)

scene.bind("mouseup", up)

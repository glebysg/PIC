from vpython import *

def draw_reference_frame(x,y,z,arrow_size=5):
    arrow(pos=vector(x,y,z), axis=vector(arrow_size,0,0), shaftwidth=1, color=color.red)
    arrow(pos=vector(x,y,z), axis=vector(0,arrow_size,0), shaftwidth=1, color=color.green)
    arrow(pos=vector(x,y,z), axis=vector(0,0,arrow_size), shaftwidth=1, color=color.blue)
    text(text='X', align='center', height=5, depth=-0.3, color=color.red,pos=vector(x+arrow_size+1,y,z))
    text(text='Y', align='center', height=5, depth=-0.3, color=color.green,pos=vector(x,y+arrow_size+1,z))
    text(text='Z', align='center', height=5, depth=-0.3, color=color.blue,pos=vector(x,y,z+arrow_size+1))

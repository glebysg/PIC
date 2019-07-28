from vpython import *
e = extrusion(path=[vec(0,0,0), vec(0,0,-5)],
    color=color.orange,
    shape=[ shapes.circle(radius=3),
            shapes.triangle(length=2),
            shapes.trapezoid(pos=[0,2], width=4,
              height=1, top=2) ]) 

print(e.__dir__())
while (True):
    rate(30)
    e.pos.x += 1

from vpython import *

a = vertex( pos=vec(0,0,0), color=color.blue)
b = vertex( pos=vec(1,0,0), color=color.cyan )
c = vertex( pos=vec(1,1,0), color=color.magenta )
d = vertex( pos=vec(0,1,0), color=color.yellow )
Q = quad( v0=a, v1=b, v2=c, v3=d )

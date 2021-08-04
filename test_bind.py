from vpython import *
import time
s = sphere(color=color.cyan)

def change():
    if s.color.equals(color.cyan):
        s.color = color.red
    else:
        s.color = color.cyan

scene.bind('click', change)
try:
    while True:
       time.sleep(1)
except KeyboardInterrupt:
    print("Press Ctrl-C to terminate while statement")
    pass

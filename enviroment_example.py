from vpython import *
#GlowScript 1.1 VPython

# This is a simple simulation ping pong as Demonstartion example. Scale is 1:10 I guees

# from visual import *

#Opening message
scene.caption = ("Hello use your mouse ball along left or right click to zoom and rotate respectively")
print(color.red)

##############################################################################################
## Drawing the Ping-pong table, racket and ball  - nothing fancy
#############################################################################################

#green table top
table_top = box (pos=vector(0,0,0), length=27.4, height=0.2, width=15.2, texture=textures.metal)

#drawing table legs just 2 legs for simplicity for illustrations purpose only
table_leg1 = box (pos = vector(13.7,-3.8,-7.6), length=0.5, height =7.6, width=0.5, texture=textures.metal)
table_leg2 = box (pos = vector(13.7,-3.8,7.6), length = 0.5, height = 7.6, width=0.5, texture=textures.metal)
table_leg3 = box (pos = vector(-13.7,-3.8,-7.6), length=0.5, height =7.6, width=0.5, texture=textures.metal)
table_leg4 = box (pos = vector(-13.7,-3.8,7.6), length = 0.5, height = 7.6, width=0.5, texture=textures.metal)

#drawing center line in the middle half the width
center_line = box (pos = vector(0, 0.1,0),length=27.4, height=0.01, width=0.5, color=color.white)

#drawing the net
# net = box (pos= vector (0,.87,0),length=0.5, height=1.52, width=15.2, color=color.yellow)

#drawing the wall at one end of the table

#drawing floor
floor = box (pos = vector(0,-7.6,0), length=35.4, height=0.5, width=18, color=vector(100/255.0,99/255.0,82/255.0))

#drawing the racket for simplicity a square plane (No regulations on shape and size in Ping Pong :)
racket_face = box (pos=vector(10,1.25,-2), length=0.5, height=2.5, width=1.5, color=color.red)
racket_face = box (pos=vector(7,1.25,-2), length=0.5, height=2.5, width=1.5, color=color.red)
racket_face = box (pos=vector(5,1.25,-2), length=0.5, height=2.5, width=1.5, color=color.red)

# ball = sphere (pos=vector(13,3,0), radius=0.2, color=color.white)

############################################################################################
## Setting up the Vectors
############################################################################################

#racket initial position
# racket_face.velocity = vector(-0.2,0,0)

#ball initial position, and vector velocity
# ball.velocity = vector(0,-1,0)

# keep these settings for simulation purpose on dampening the motion in the y axis
# dt = 0.01


##############################################################################################
## Simulation loop
##############################################################################################
while 1:

    #sets the pace
    rate (75)
    # ball.pos = ball.pos + ball.velocity*dt
    # racket_face.pos = racket_face.pos + racket_face.velocity*dt*20
 
    # #bouncing
    # if ball.pos.y < ball.radius:
        # ball.velocity.y = abs(ball.velocity.y)
    # else:
        # ball.velocity.y = ball.velocity.y - 9.8*dt

    # #if collision occurs, momentum of racket ball
    # if ball.pos.x <= racket_face.pos.x:
        # ball.velocity.x = ball.velocity.x + racket_face.velocity.x 
        
    
    # #bounce if hits the net
    # if (ball.pos.x < net.pos.x) and (ball.pos.y < net.pos.y):
        # ball.velocity.x = abs(ball.velocity.x)
    
    # #stop the racket
    # if racket_face.pos.x < 10:
        # racket_face.velocity = vector(0,0,0)
    
    # #if hits the ball bounce
    # if ball.pos.x < wall.pos.x:                         
        # ball.velocity.x = abs(ball.velocity.x)
        
    # #if outbounds ball stops    
    # if ball.pos.x > 15:        
        # ball.velocity = vector(0,0,0)
        

        
    
        
        



   

    

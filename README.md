# PIC

Pose-Imitation Constraint (PIC) algorithm that can be used on any 
kinematic chain (with two or more links) 

The PIC algorithm is based on the FABRIK algorithm. http://www.andreasaristidou.com/FABRIK.html.

# USAGE
### Step 1 - Create a robot
To add a new robot, first create  a DH parameters file. 
The file must be of type .csv with space as separators. The columns of the .csv must follow the format below: 

![](/Users/glebysgonzalez/Purdue/py-FABRIK/Media/DH_table.png)

You can find a sample for the Baxter robot under: `simulation/arms/baxter_left`.csv and `simulation/arms/baxter_right.csv`

import numpy as np
from os import path
tasks = ['incision_curvy', 'incision_straight']
robots = ["yumi","baxter"]
algorithms = ['fabrik', 'poseimit0', 'poseimit3']
out_path = 'data/results/'
pose_file = "pose.txt"
occlussion_file = "occlussion.txt"
distance_file = "distances.txt"
pose_writer = open(out_path+pose_file,"w+")
occlussion_writer = open(out_path+occlussion_file, "w+")
distance_writer = open(out_path+distance_file, "w+")
show_std = True
dec = "{:.2f}"

for task in tasks:
    for robot in robots:
        pose_line = ""
        occ_line = ""
        dist_line = ""
        h_occ_mean = ""
        h_occ_std = ""
        for algorithm in algorithms:
            name = out_path+task+"_"+ robot + "_" + algorithm + ".txt"
            if path.exists(name):
                print("Processing:", name)
                result = np.loadtxt(name, delimiter=' ')
                result[:,2] = result[:,2]
                result[:,1] = result[:,1]
                means = np.mean(result, axis=0)
                stds = np.std(result, axis=0)
                pose_line += dec.format(means[0])
                occ_line += dec.format(means[2])
                dist_line += dec.format(means[3])
                h_occ_mean = dec.format(means[1])
                h_occ_std = dec.format(stds[1])
                if show_std:
                    pose_line += " ("+ dec.format(stds[0]) +")"
                    occ_line += " ("+ dec.format(stds[2]) +")"
                    dist_line += " ("+ dec.format(stds[3]) +")"
                pose_line += " & "
                occ_line += " & "
                dist_line += " & "
        pose_line = pose_line[:-2]
        occ_line = occ_line[:-2]
        dist_line = dist_line[:-2]
        occ_line += " & " + h_occ_mean
        if show_std:
            occ_line += " ("+ h_occ_std +")"
        pose_line += "\\\\\n"
        occ_line += "\\\\\n"
        dist_line += "\\\\\n"
        pose_writer.writelines(pose_line)
        occlussion_writer.writelines(occ_line)
        distance_writer.writelines(dist_line)
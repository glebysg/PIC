import numpy as np
from os import path
tasks = ['incision_curvy1', 'incision_straight3', 'assembly1', 'assembly2', 'assembly3']
robots = ["yumi","baxter"]
# algorithms = ['fabrik', 'poseimit0', 'poseimit3']
algorithms = ['fabrik', 'posecones']
out_path = 'data/unfiltered_results/'
# out_path = 'data/rss_results/'
pose_file = "pose.txt"
occlussion_file = "occlussion.txt"
distance_file = "distances.txt"
pose_writer = open(path.join(out_path,pose_file),"w+")
occlussion_writer = open(path.join(out_path+occlussion_file), "w+")
distance_writer = open(path.join(out_path+distance_file), "w+")
show_std = True
dec = "{:.2f}"
task_count=0
for task in tasks:
    for robot in robots:
        pose_line = ""
        occ_line = ""
        dist_line = ""
        h_occ_mean = ""
        h_occ_std = ""
        for algorithm in algorithms:
            name = path.join(out_path,task+"_"+ robot + "_" + algorithm + ".txt")
            if path.exists(name):
                print("Processing:", name)
                result = np.loadtxt(name, delimiter=' ')
                if "assembly" in task:
                    result[:,2] = 1- result[:,2]
                    result[:,1] = 1- result[:,1]
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

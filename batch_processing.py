import numpy as np
from os import path

task_groups = [['incision_straight3', 'incision_curvy1'], ['assembly1', 'assembly2', 'assembly3']]
task_groups = [['incision_straight3', 'incision_curvy1']]
task_groups = [['assembly1', 'assembly2', 'assembly3']]
robots = ["yumi","baxter"]
# algorithms = ['fabrik', 'poseimit0', 'poseimit3']
# algorithms = ['fabrik', 'poseimit0','poseimit1','poseimit2', 'poseimit3']
# algorithms = ['fabrik','posecones']
algorithms = ['posecones']
# out_path = 'data/rss_results/'
# out_path = 'data/unfiltered_results/'
# out_path = 'iros_results/soft_to_rigid_20/'
out_path = 'data/iros_rigid_to_soft_20/'
out_path = 'data/iros_soft_to_rigid_20/'
# out_path = 'data/iros_rigid_20/'
pose_file = "pose_table_full.txt"
occlussion_file = "occlussion_table_full.txt"
distance_file = "distances_table_full.txt"
pose_writer = open(path.join(out_path,pose_file),"w+")
occlussion_writer = open(path.join(out_path,occlussion_file), "w+")
distance_writer = open(path.join(out_path,distance_file), "w+")
show_std = True
dec = "{:.2f}"
pose_totals = [[] for i in range(len(algorithms))]
dist_totals = [[] for i in range(len(algorithms))]
occ_totals = [[] for i in range(len(algorithms))]
hum_totals = [[] for i in range(len(algorithms))]
task_count=0
for tasks in task_groups:
    for robot in robots:
        for task in tasks:
            pose_line = ""
            occ_line = ""
            dist_line = ""
            h_occ_mean = ""
            h_occ_std = ""
            algorithm_index = 0
            for algorithm in algorithms:
                name = path.join(out_path,task+"_"+ robot + "_" + algorithm + ".txt")
                if path.exists(name):
                    print("Processing:", name)
                    result = np.loadtxt(name, delimiter=' ')
                    # unify the occlussion metrics
                    # if "assembly" in task:
                        # result[:,2] = 1- result[:,2]
                        # result[:,1] = 1- result[:,1]
                    # accumulated results in the totals matrix
                    pose_totals[algorithm_index] = np.concatenate((pose_totals[algorithm_index],result[:,0]))
                    hum_totals[algorithm_index] = np.concatenate((hum_totals[algorithm_index],result[:,1]))
                    occ_totals[algorithm_index] = np.concatenate((occ_totals[algorithm_index],result[:,2]))
                    dist_totals[algorithm_index] = np.concatenate((dist_totals[algorithm_index],result[:,3]))
                    #### finished accumulating
                    means = np.mean(result, axis=0)
                    stds = np.std(result, axis=0)
                    pose_line += dec.format(means[0])
                    h_occ_mean = dec.format(means[1])
                    h_occ_std = dec.format(stds[1])
                    occ_line += dec.format(means[2])
                    dist_line += dec.format(means[3])
                    if show_std:
                        pose_line += " ("+ dec.format(stds[0]) +")"
                        occ_line += " ("+ dec.format(stds[2]) +")"
                        dist_line += " ("+ dec.format(stds[3]) +")"
                    pose_line += " & "
                    occ_line += " & "
                    dist_line += " & "
                    algorithm_index += 1
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
# Do the totals
pose_line = ""
occ_line = ""
dist_line = ""
for algorithm_index in range(len(algorithms)):
    pose_line += dec.format(np.mean(pose_totals[algorithm_index]))
    occ_line += dec.format(np.mean(occ_totals[algorithm_index]))
    dist_line += dec.format(np.mean(dist_totals[algorithm_index]))
    if show_std:
        pose_line += " ("+ dec.format(np.std(pose_totals[algorithm_index])) +")"
        occ_line += " ("+ dec.format(np.std(occ_totals[algorithm_index])) +")"
        dist_line += " ("+ dec.format(np.std(dist_totals[algorithm_index])) +")"
    pose_line += " & "
    occ_line += " & "
    dist_line += " & "
pose_line = pose_line[:-2]
occ_line = occ_line[:-2]
dist_line = dist_line[:-2]

flat_hum_totals = []
print(hum_totals)
for hum_line in hum_totals:
    flat_hum_totals += list(hum_line.astype(float))

occ_line += " & " + dec.format(np.mean(flat_hum_totals))
if show_std:
    occ_line += " ("+ dec.format(np.std(flat_hum_totals)) +")"
pose_line += "\\\\\n"
occ_line += "\\\\\n"
dist_line += "\\\\\n"
pose_writer.writelines(pose_line)
occlussion_writer.writelines(occ_line)
distance_writer.writelines(dist_line)
pose_writer.close()
occlussion_writer.close()
distance_writer.close()

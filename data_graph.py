import numpy as np
from os import path
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

def autolabel(rects):
    for rect in rects:
        h = rect.get_height()
        ax.text(rect.get_x()+rect.get_width()/2., 1.05*h, '%d'%int(h),
                ha='center', va='bottom')

incision_tasks = ['incision_straight3', 'incision_curvy1']
assembly_tasks = ['assembly1','assebly2','assembly3']
robots = ["yumi","baxter"]
# algorithms = ['fabrik','poseimit0', 'poseimit1', 'poseimit2', 'poseimit3']
algorithms = ['fabrik','poseimit0', 'poseimit1', 'poseimit2', 'poseimit3']
# out_path = 'data/rss_results/'
out_path = 'data/unfiltered_results/'
pose_file = "pose.txt"
occlussion_file = "occlussion.txt"
distance_file = "distances.txt"
show_std = True
dec = "{:.2f}"
show_std = False

################# CALCULATE THE INCISION ########################

inc_pose_means = []
inc_pose_stds = []
inc_dist_means = []
inc_dist_stds = []
inc_occ_means = []
inc_occ_stds = []

for algorithm in algorithms:
    full_result = None
    for task in incision_tasks:
        for robot in robots:
                name = out_path+task+"_"+ robot + "_" + algorithm + ".txt"
                if path.exists(name):
                    if full_result is None:
                        full_result = np.loadtxt(name, delimiter=' ')
                        # full_result[:,2] = full_result[:,2]
                    else:
                        result = np.loadtxt(name, delimiter=' ')
                        # result[:,2] = result[:,2]
                        full_result = np.concatenate((result,full_result),axis=0)
    # PLOT THE RESULT
    # Calculate the average and standard deviation
    means = np.mean(full_result,axis=0)
    stds = np.std(full_result,axis=0)
    inc_pose_means.append(means[0])
    inc_pose_stds.append(stds[0])
    inc_dist_means.append(means[3])
    inc_dist_stds.append(stds[3])
    inc_occ_means.append(means[2])
    inc_occ_stds.append(stds[2])

print(inc_occ_means)
print(inc_occ_stds)

################# CALCULATE THE ASSEMBLY ########################
asb_pose_means = []
asb_pose_stds = []
asb_dist_means = []
asb_dist_stds = []
asb_occ_means = []
asb_occ_stds = []

for algorithm in algorithms:
    full_result = None
    for task in assembly_tasks:
        for robot in robots:
                name = out_path+task+"_"+ robot + "_" + algorithm + ".txt"
                if path.exists(name):
                    print("Processing:", name)
                    if full_result is None:
                        full_result = np.loadtxt(name, delimiter=' ')
                        # full_result[:,2] = full_result[:,2]
                        print(full_result)
                    else:
                        result = np.loadtxt(name, delimiter=' ')
                        # result[:,2] = result[:,2]
                        full_result = np.concatenate((result,full_result),axis=0)
    # PLOT THE RESULT
    # Calculate the average and standard deviation
    means = np.mean(full_result,axis=0)
    stds = np.std(full_result,axis=0)
    asb_pose_means.append(means[0])
    asb_pose_stds.append(stds[0])
    asb_dist_means.append(means[3])
    asb_dist_stds.append(stds[3])
    asb_occ_means.append(means[2])
    asb_occ_stds.append(stds[2])

print(asb_occ_means)
print(asb_occ_stds)

#################### OCCLUSSION #########################
# Create lists for the plot
x_pos = np.arange(len(algorithms))
width = 0.25
inc_CTEs = inc_occ_means
inc_error = inc_occ_stds
asb_CTEs = asb_occ_means
asb_error = asb_occ_stds

# Build the plot
fig, ax = plt.subplots()
if show_std:
    barplot_1 = ax.bar(x_pos, inc_CTEs, width,  yerr=inc_error, alpha=0.65, ecolor='black', edgecolor='black', linewidth=2, capsize=10, color='yellow')
    barplot_2 = ax.bar(x_pos + width + width*0.2, asb_CTEs, width,  yerr=asb_error, alpha=0.65, ecolor='black', edgecolor='black', linewidth=3, capsize=10, color='darkblue')
else:
    barplot_1 = ax.bar(x_pos, inc_CTEs, width,  alpha=0.65, ecolor='black', edgecolor='black', linewidth=2, capsize=10, color='yellow')
    barplot_2 = ax.bar(x_pos + width + width*0.2, asb_CTEs, width,  alpha=0.65, ecolor='black', edgecolor='black', linewidth=3, capsize=10, color='darkblue')
# barplot[0].set_color('yellow')
# barplot[1].set_color('coral')
# barplot[2].set_color('crimson')
# barplot[3].set_color('darkmagenta')
# barplot[4].set_color('darkblue')
ax.set_ylabel('Percentage of Occlussion/Obstuction (PO)')
ax.set_xticks(x_pos+width)
ax.set_xticklabels(['FABRIK','PIC','PICs \u03B7=1', 'PICs \u03B7=2', 'PICs \u03B7=3'])
ax.set_title('PO for different levels of softening')
ax.yaxis.grid(True)
ax.legend( (barplot_1[0], barplot_2[0]), ('Incision', 'Assembly') )

# Save the figure and show
plt.tight_layout()
plt.savefig(out_path+'occ_graph.png')
plt.show()

#################### ERROR DISTANCE #########################
# Create lists for the plot
x_pos = np.arange(len(algorithms))
width = 0.25
inc_CTEs = inc_dist_means
inc_error = inc_dist_stds
asb_CTEs = asb_dist_means
asb_error = asb_dist_stds

# Build the plot
fig, ax = plt.subplots()
if show_std:
    barplot_1 = ax.bar(x_pos, inc_CTEs, width,  yerr=inc_error, alpha=0.65, ecolor='black', edgecolor='black', linewidth=2, capsize=10, color='yellow')
    barplot_2 = ax.bar(x_pos + width + width*0.2, asb_CTEs, width,  yerr=asb_error, alpha=0.65, ecolor='black', edgecolor='black', linewidth=3, capsize=10, color='darkblue')
else:
    barplot_1 = ax.bar(x_pos, inc_CTEs, width,  alpha=0.65, ecolor='black', edgecolor='black', linewidth=2, capsize=10, color='yellow')
    barplot_2 = ax.bar(x_pos + width + width*0.2, asb_CTEs, width,  alpha=0.65, ecolor='black', edgecolor='black', linewidth=3, capsize=10, color='darkblue')
# barplot[0].set_color('yellow')
# barplot[1].set_color('coral')
# barplot[2].set_color('crimson')
# barplot[3].set_color('darkmagenta')
# barplot[4].set_color('darkblue')
ax.set_ylabel('MSE (cm)')
ax.set_xticks(x_pos)
ax.set_xticklabels(['FABRIK','PIC','PICs \u03B7=1', 'PICs \u03B7=2', 'PICs \u03B7=3'])
ax.set_title('MSE between human and robot targets for different levels of softening')
ax.yaxis.grid(True)

# Save the figure and show
plt.tight_layout()
plt.savefig(out_path+'dist_graph.png')
plt.show()

#################### POSE IMITATION #########################
inc_CTEs = inc_pose_means
inc_error = inc_pose_stds
asb_CTEs = asb_pose_means
asb_error = asb_pose_stds

# Build the plot
fig, ax = plt.subplots()
if show_std:
    barplot_1 = ax.bar(x_pos, inc_CTEs, width,  yerr=inc_error, alpha=0.65, ecolor='black', edgecolor='black', linewidth=2, capsize=10, color='yellow')
    barplot_2 = ax.bar(x_pos + width + width*0.2, asb_CTEs, width,  yerr=asb_error, alpha=0.65, ecolor='black', edgecolor='black', linewidth=3, capsize=10, color='darkblue')
else:
    barplot_1 = ax.bar(x_pos, inc_CTEs, width,  alpha=0.65, ecolor='black', edgecolor='black', linewidth=2, capsize=10, color='yellow')
    barplot_2 = ax.bar(x_pos + width + width*0.2, asb_CTEs, width,  alpha=0.65, ecolor='black', edgecolor='black', linewidth=3, capsize=10, color='darkblue')
# barplot[0].set_color('yellow')
# barplot[1].set_color('coral')
# barplot[2].set_color('crimson')
# barplot[3].set_color('darkmagenta')
# barplot[4].set_color('darkblue')
ax.set_ylabel('Pose Accuracy (Pacc) w.r.t the human')
ax.set_xticks(x_pos+width)
ax.set_xticks(x_pos)
ax.set_xticklabels(['FABRIK','PIC','PICs \u03B7=1', 'PICs \u03B7=2', 'PICs \u03B7=3'])
# ax.set_title('Pose similarity with the human for different levels of softening')
ax.yaxis.grid(True)
ax.legend( (barplot_1[0], barplot_2[0]), ('Incision', 'Assembly') )

# Save the figure and show
plt.tight_layout()
plt.savefig(out_path+'pose_graph.png')
plt.show()

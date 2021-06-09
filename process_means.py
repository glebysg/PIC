import numpy as np
from os import path
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

def autolabel(rects):
    for rect in rects:
        h = rect.get_height()
        ax.text(rect.get_x()+rect.get_width()/2., 1.05*h, '%d'%int(h),
                ha='center', va='bottom')

tasks = ['incision_straight3', 'incision_curvy1','assembly1','assembly2','assembly3']
tasks_names = ['Incision-S', 'Incision-C','Assembly 1','Assembly 2','Assembly 3']
# tasks = ['assembly1','assembly2','assembly3']
# tasks = ['assembly3']
# tasks = ['incision_straight3']
robots = ["yumi"]
# algorithms = ['fabrik','poseimit0', 'poseimit1', 'poseimit2', 'poseimit3']
algorithms = ['fabrik']
# out_path = 'data/rss_results/'
out_path = 'data/human_processing/'
pose_file = "angles_raw.txt"
show_std = True
dec = "{:.2f}"
show_std = False

shoulder_stds = []
elbow_stds = []

################# CALCULATE THE INCISION ########################
all_results = None 
for algorithm in algorithms:
    full_result = None
    for task in tasks:
        for robot in robots:
                name = out_path+task+"_"+ robot + "_" + algorithm + "_RAW.txt"
                print("name:", name)
                if path.exists(name):
                    full_result = np.loadtxt(name, delimiter=' ')
                    # PLOT THE RESULT
                    # Calculate the average and standard deviation
                    filtered_mask = [~np.isnan(full_result).any(axis=1)]
                    if all_results is None:
                        all_results = full_result
                    else:
                        all_results = np.concatenate((all_results, full_result), axis=0)
                    means = np.mean(full_result[filtered_mask],axis=0)
                    stds = np.std(full_result[filtered_mask],axis=0)
                    shoulder_means  = means[0]
                    shoulder_stds.append(stds[0])
                    elbow_means = means[1]
                    elbow_stds.append(stds[1])
                    print(shoulder_means, shoulder_stds[-1])
                    print(elbow_means, elbow_stds[-1])
print("TOTAL:")
print(np.mean(all_results[:,0]), np.std(all_results[:,0]))
print(np.mean(all_results[:,1]), np.std(all_results[:,1]))

#################### GRAPH  #########################
# Create lists for the plot
x_pos = np.arange(len(tasks_names))
width = 0.25
shoulder_stds_deg = np.degrees(shoulder_stds)
elbow_stds_deg = np.degrees(elbow_stds)

# Build the plot
fig, ax = plt.subplots()
if show_std:
    barplot_1 = ax.bar(x_pos, shoulder_stds_deg, width,  alpha=0.80, ecolor='black', edgecolor='black', linewidth=2, capsize=10, color='darkgreen')
    barplot_2 = ax.bar(x_pos + width + width*0.2, elbow_stds_deg, width,  alpha=0.80, ecolor='black', edgecolor='black', linewidth=3, capsize=10, color='purple')
else:
    barplot_1 = ax.bar(x_pos, shoulder_stds_deg, width,  alpha=0.80, ecolor='black', edgecolor='black', linewidth=2, capsize=10, color='darkgreen')
    barplot_2 = ax.bar(x_pos + width + width*0.2, elbow_stds_deg, width,  alpha=0.80, ecolor='black', edgecolor='black', linewidth=3, capsize=10, color='purple')
# barplot[0].set_color('yellow')
# barplot[1].set_color('coral')
# barplot[2].set_color('crimson')
# barplot[3].set_color('darkmagenta')
# barplot[4].set_color('darkblue')
ax.set_ylabel('Motion variance (Degrees)')
ax.set_xticks(x_pos+width)
ax.set_xticklabels(['Incision-S', 'Incision-C','Assembly 1','Assembly 2','Assembly 3'])
ax.set_title('Motion variance in shoulder and elbow (Degrees)')
ax.yaxis.grid(True)
ax.legend( (barplot_1[0], barplot_2[0]), ('Shoulder', 'Elbow') )

# Save the figure and show
plt.tight_layout()
plt.savefig(out_path+'MOTION_graph.png')
plt.show()


exit()

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

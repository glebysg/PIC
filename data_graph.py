import numpy as np
from os import path
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

tasks = ['incision_curvy', 'incision_straight']
robots = ["yumi","baxter"]
algorithms = ['fabrik','poseimit0', 'poseimit1', 'poseimit2', 'poseimit3']
out_path = 'data/results/'
pose_file = "pose.txt"
occlussion_file = "occlussion.txt"
distance_file = "distances.txt"
pose_writer = open(out_path+pose_file,"w+")
occlussion_writer = open(out_path+occlussion_file, "w+")
distance_writer = open(out_path+distance_file, "w+")
show_std = True
dec = "{:.2f}"

pose_means = []
pose_stds = []
dist_means = []
dist_stds = []
occ_means = []
occ_stds = []

for algorithm in algorithms:
    full_result = None
    for task in tasks:
        for robot in robots:
                name = out_path+task+"_"+ robot + "_" + algorithm + ".txt"
                if path.exists(name):
                    print("Processing:", name)
                    if full_result is None:
                        full_result = np.loadtxt(name, delimiter=' ')
                        full_result[:,2] = full_result[:,2]
                    else:
                        result = np.loadtxt(name, delimiter=' ')
                        result[:,2] = result[:,2]
                        full_result = np.concatenate((result,full_result),axis=0)
    # PLOT THE RESULT
    # Calculate the average and standard deviation
    means = np.mean(full_result,axis=0)
    stds = np.std(full_result,axis=0)
    pose_means.append(means[0])
    pose_stds.append(stds[0])
    dist_means.append(means[3])
    dist_stds.append(stds[3])
    occ_means.append(means[2])
    occ_stds.append(stds[2])

print(occ_means)
print(occ_stds)
# Create lists for the plot
x_pos = np.arange(len(algorithms))
CTEs = occ_means
error = occ_stds


# Build the plot
fig, ax = plt.subplots()
barplot = ax.bar(x_pos, CTEs, yerr=error, align='center', alpha=0.65, ecolor='black', edgecolor='black', linewidth=2, capsize=10)
barplot[0].set_color('yellow')
barplot[1].set_color('coral')
barplot[2].set_color('crimson')
barplot[3].set_color('darkmagenta')
barplot[4].set_color('darkblue')
ax.set_ylabel('Percentage of visible pad')
ax.set_xticks(x_pos)
ax.set_xticklabels(['FABRIK','PIC','PICs \u03B7=1', 'PICs \u03B7=2', 'PICs \u03B7=3'])
ax.set_title('POA for different levels of softening')
ax.yaxis.grid(True)

# Save the figure and show
plt.tight_layout()
plt.savefig(out_path+'occ_graph.png')
plt.show()

# Create lists for the plot
x_pos = np.arange(len(algorithms))
CTEs = dist_means
error = dist_stds

# Build the plot
fig, ax = plt.subplots()
barplot = ax.bar(x_pos, CTEs, yerr=error, align='center', alpha=0.65, ecolor='black', edgecolor='black', linewidth=2, capsize=10)
barplot[0].set_color('yellow')
barplot[1].set_color('coral')
barplot[2].set_color('crimson')
barplot[3].set_color('darkmagenta')
barplot[4].set_color('darkblue')
ax.set_ylabel('MSE (cm)')
ax.set_xticks(x_pos)
ax.set_xticklabels(['FABRIK','PIC','PICs \u03B7=1', 'PICs \u03B7=2', 'PICs \u03B7=3'])
ax.set_title('MSE between human and robot targets for different levels of softening')
ax.yaxis.grid(True)

# Save the figure and show
plt.tight_layout()
plt.savefig(out_path+'dist_graph.png')
plt.show()

# Create lists for the plot
x_pos = np.arange(len(algorithms))
CTEs = pose_means
error = pose_stds
print(pose_means)
print(pose_stds)

# Build the plot
fig, ax = plt.subplots()
barplot = ax.bar(x_pos, CTEs, yerr=error, align='center', alpha=0.65, ecolor='black', edgecolor='black', linewidth=2, capsize=10)
barplot[0].set_color('yellow')
barplot[1].set_color('coral')
barplot[2].set_color('crimson')
barplot[3].set_color('darkmagenta')
barplot[4].set_color('darkblue')
ax.set_ylabel('Pose imitation accuracy')
ax.set_xticks(x_pos)
ax.set_xticklabels(['FABRIK','PIC','PICs \u03B7=1', 'PICs \u03B7=2', 'PICs \u03B7=3'])
ax.set_title('Pose Imitation Accuracy  with different levels of softening')
ax.yaxis.grid(True)

# Save the figure and show
plt.tight_layout()
plt.savefig(out_path+'pose_graph.png')
plt.show()

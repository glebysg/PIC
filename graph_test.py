import seaborn as sns
sns.set(style="darkgrid")
tips = sns.load_dataset("tips")
print(tips)
color = sns.color_palette()[2]
g = sns.jointplot("total_bill", "tip", data=tips, kind="reg",
                  xlim=(0, 60), ylim=(0, 12), color=color, size=7)
import matplotlib.pyplot as plt
plt.show()

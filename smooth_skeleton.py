import numpy as np
from statsmodels.tsa.api import ExponentialSmoothing, SimpleExpSmoothing, Holt
import os

# init vars
alpha = 0.6
read_dir = './data/new/'

for filename in os.listdir(read_dir):
    if filename.endswith("skel.txt"):
        print("Processing file: ",filename)
        skel_reader = open(os.path.join(read_dir,filename), 'r')
        write_path = os.path.join(read_dir,'smooth_'+filename)
        full_data = []
        for line in skel_reader:
            data_point = np.array(line.split(), dtype=float)
            full_data.append(data_point)
        full_data = np.array(full_data)
        print(full_data.shape)

        smooth_data = []
        rows, cols = full_data.shape
        # smooth data column by column
        for c in range(cols):
            smooth_col = SimpleExpSmoothing(full_data[:,c]).fit(smoothing_level=alpha,optimized=False)
            smooth_data.append(np.array(smooth_col.fittedvalues))
        smooth_data = (np.transpose(np.array(smooth_data))).reshape(rows,cols)
        np.savetxt(write_path, smooth_data, delimiter=' ')

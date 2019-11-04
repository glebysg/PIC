import numpy as np
from statsmodels.tsa.api import ExponentialSmoothing, SimpleExpSmoothing, Holt
import os
from helpers import kalman_filter

# init vars
alpha = 0.6
kalman_Q = 1e-5 # process variance
read_dir = './data/new/'

for filename in os.listdir(read_dir):
    if filename.endswith("skel.txt") and ("smooth" not in filename):
        print("Processing file: ",filename)
        skel_reader = open(os.path.join(read_dir,filename), 'r')
        write_path = os.path.join(read_dir,'smooth_'+filename)
        kalman_write_path = os.path.join(read_dir,'kalman_smooth_'+filename)
        double_write_path = os.path.join(read_dir,'double_smooth_'+filename)
        full_data = []
        for line in skel_reader:
            data_point = np.array(line.split(), dtype=float)
            full_data.append(data_point)
        full_data = np.array(full_data)
        print(full_data.shape)

        smooth_data = []
        kalman_smooth_data = []
        double_smooth_data = []
        rows, cols = full_data.shape
        # smooth data column by column
        for c in range(cols):
            smooth_col = SimpleExpSmoothing(full_data[:,c]).fit(smoothing_level=alpha,optimized=False)
            smooth_col = np.array(smooth_col.fittedvalues)
            smooth_data.append(smooth_col)
            kalman_smooth_col, pminus = kalman_filter(full_data[:,c], kalman_Q)
            kalman_smooth_data.append(np.array(kalman_smooth_col))
            double_smooth_col, pminus = kalman_filter(smooth_col[:], kalman_Q)
            double_smooth_data.append(np.array(double_smooth_col))
        smooth_data = (np.transpose(np.array(smooth_data))).reshape(rows,cols)
        kalman_smooth_data = (np.transpose(np.array(kalman_smooth_data))).reshape(rows,cols)
        double_smooth_data = (np.transpose(np.array(double_smooth_data))).reshape(rows,cols)
        np.savetxt(write_path, smooth_data, delimiter=' ')
        np.savetxt(kalman_write_path, kalman_smooth_data, delimiter=' ')
        np.savetxt(double_write_path, double_smooth_data, delimiter=' ')

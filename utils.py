import numpy as np
def normalize(vector):
    return np.array(vector,dtype=float)/np.linalg.norm(vector)

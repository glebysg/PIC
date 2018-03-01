import numpy as np
def normalize(vector):
    return np.array(vector,dtype=float)/np.linalg.norm(vector)

def distance(vector1, vector2):
    return np.linalg.norm(vector1-vector2)

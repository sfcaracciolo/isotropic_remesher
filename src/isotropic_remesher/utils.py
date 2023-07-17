import numpy as np 
from open3d.utility import Vector3dVector 
from open3d.geometry import PointCloud 

def compute_len_stats(vertices:Vector3dVector):
    pcd = PointCloud(vertices)
    d = np.asarray(pcd.compute_nearest_neighbor_distance())
    return d.min(), d.mean(), d.max()

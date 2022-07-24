import numpy as np
import open3d as o3d

# create test data
points = np.random.uniform(-1, 1, size=(100, 3))
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# threshold data
points = np.asarray(pcd.points)
pcd_sel = pcd.select_down_sample(np.where(points[:, 2] > 0)[0])



# visualize different point clouds
o3d.visualization.draw_geometries([pcd])
o3d.visualization.draw_geometries([pcd_sel])
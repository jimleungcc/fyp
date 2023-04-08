import open3d as o3d
import numpy as np

pcd1 = o3d.io.read_point_cloud("door.ply")
pcd2 = o3d.io.read_point_cloud("Area_5/table.ply")

# Compute point-to-point distances
distance = pcd1.compute_point_cloud_distance(pcd2)

# Find indices of closest points
min_index_pcd1 = np.argmin(distance, axis=0)
min_index_pcd2 = np.argmin(distance, axis=1)

# Get closest points
closest_point_pcd1 = pcd1.points[min_index_pcd1]
closest_point_pcd2 = pcd2.points[min_index_pcd2]

print("Closest point in pcd1:", closest_point_pcd1)
print("Closest point in pcd2:", closest_point_pcd2)

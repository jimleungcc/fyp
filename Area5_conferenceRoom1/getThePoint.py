import open3d as o3d
import numpy as np

# Load the point clouds
pcd1 = o3d.io.read_point_cloud("westwall_combine.ply")
pcd2 = o3d.io.read_point_cloud("westchair1.ply")

# Convert the point clouds to numpy arrays
pcd1_np = np.asarray(pcd1.points)
pcd2_np = np.asarray(pcd2.points)

# Compute the pairwise distances between the points in the two point clouds
distances = o3d.geometry.PointCloud.compute_point_cloud_distance(pcd1, pcd2)
# Find the indices of the closest points in pcd2 for each point in pcd1
closest_indices = np.argmin(distances, axis=0)
# Use the closest_indices array to get the coordinates of the closest points in pcd2 for each point in pcd1
closest_points = pcd1_np[closest_indices]
print(closest_points)
point1 = o3d.io.read_point_cloud("point1.ply")

distances = o3d.geometry.PointCloud.compute_point_cloud_distance(pcd2, pcd1)
closest_indices = np.argmin(distances, axis=0)
closest_points = pcd2_np[closest_indices]
print(closest_points)
point2 = o3d.io.read_point_cloud("point2.ply")

print(np.min(distances))





objects = [];
objects.append(pcd1)
objects.append(pcd2)
objects.append(point1)
objects.append(point2)
o3d.visualization.draw_geometries(objects)
import open3d as o3d
import numpy as np

# Load the point clouds
pcd1 = o3d.io.read_point_cloud("Area_5/westwall_combine.ply")
pcd2 = o3d.io.read_point_cloud("Area_5/westchair1.ply")

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

fromWall = closest_points


distances = o3d.geometry.PointCloud.compute_point_cloud_distance(pcd2, pcd1)
closest_indices = np.argmin(distances, axis=0)
closest_points = pcd2_np[closest_indices]
print(closest_points)
point2 = o3d.io.read_point_cloud("point2.ply")

fromObj = closest_points

print(np.min(distances))

line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(np.array([fromWall, fromObj]))
line_set.lines = o3d.utility.Vector2iVector(np.array([[0,1]]))


objects = [];
objects.append(pcd1)
objects.append(pcd2)
objects.append(point1)
objects.append(point2)
objects.append(line_set)
o3d.visualization.draw_geometries(objects)
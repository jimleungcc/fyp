# import open3d as o3d
# import numpy as np
#
# pcd1 = o3d.io.read_point_cloud("door.ply")
# pcd2 = o3d.io.read_point_cloud("table.ply")
#
# pcd1 = pcd1.voxel_down_sample(voxel_size = 0.05)
# pcd2 = pcd2.voxel_down_sample(voxel_size = 0.05)
#
# distance = pcd1.compute_point_cloud_distance(pcd2)
# distance = np.asarray(distance)
#
# print(distance);
#
# # Find indices of closest points
# min_index_pcd1 = np.argmin(distance, axis=0)
# min_index_pcd2 = np.argmin(distance, axis=0)
#
# print("min_index1:", min_index_pcd1)
# print("min_index2:", min_index_pcd2)
#
# # Get closest points
# closest_point_pcd1 = pcd1.points[min_index_pcd1]
# closest_point_pcd2 = pcd2.points[min_index_pcd2]
#
# print("Closest point in pcd1:", closest_point_pcd1)
# print("Closest point in pcd2:", closest_point_pcd2)
#
# # Create line set from the closest points
# line_set = o3d.geometry.LineSet()
# line_set.points = o3d.utility.Vector3dVector([closest_point_pcd1, closest_point_pcd2])
# line_set.lines = o3d.utility.Vector2iVector([[0, 1]])
#
#
# objects = [];
# objects.append(pcd1)
# objects.append(pcd2)
# objects.append(line_set)
#
# o3d.visualization.draw_geometries(objects)

import open3d as o3d
import numpy as np

# Load the point clouds
pcd1 = o3d.io.read_point_cloud("westchair1.ply")
pcd2 = o3d.io.read_point_cloud("westwall_combine.ply")

pcd1 = pcd1.voxel_down_sample(voxel_size = 0.05)
pcd2 = pcd2.voxel_down_sample(voxel_size = 0.05)

# Convert the point clouds to numpy arrays
pcd1_np = np.asarray(pcd1.points)
pcd2_np = np.asarray(pcd2.points)

# Compute the pairwise distances between the points in the two point clouds
distances = o3d.geometry.PointCloud.compute_point_cloud_distance(pcd1, pcd2)

closest_indices = np.argmin(distances, axis=0)
closest_point1 = pcd2_np[closest_indices]

distances = o3d.geometry.PointCloud.compute_point_cloud_distance(pcd2, pcd1)
closest_point2 = pcd1_np[closest_indices]

print("point1:", closest_point1)
print("point2:", closest_point2)

# Find the minimum distance in the distances array
# min_distance = np.min(distances)
# print(min_distance)

line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector([closest_point1, closest_point2])
line_set.lines = o3d.utility.Vector2iVector([[0, 1]])

point1 = o3d.io.read_point_cloud("point1.ply")
point2 = o3d.io.read_point_cloud("point2.ply")


objects = [];
objects.append(pcd1)
objects.append(pcd2)
objects.append(line_set)
objects.append(point1)
objects.append(point2)
o3d.visualization.draw_geometries(objects)
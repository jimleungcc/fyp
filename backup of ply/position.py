import open3d as o3d
import numpy as np

# Load the PLY file
pcd1 = o3d.io.read_point_cloud("eastchair1.ply")
pcd2 = o3d.io.read_point_cloud("westchair4.ply")
pcd3 = o3d.io.read_point_cloud("eastchair2.ply")
pcd4 = o3d.io.read_point_cloud("westchair3.ply")
pcd5 = o3d.io.read_point_cloud("westchair1.ply")
pcd6 = o3d.io.read_point_cloud("table.ply")

# # Define the desired position as a numpy array
# pcd1_position = np.array([0, 0, 0])
# pcd2_position = np.array([0, 0, 0])
# pcd3_position = np.array([0, 0, 0])
# pcd4_position = np.array([0, 0, 0])
# pcd5_position = np.array([0, 0, 0])
# pcd6_position = np.array([0, 0, 0])
#
# # Translate the point cloud to the desired position
# pcd1.translate(pcd1_position)
# pcd2.translate(pcd2_position)
# pcd3.translate(pcd3_position)
# pcd4.translate(pcd4_position)
# pcd5.translate(pcd5_position)

# Create an axis-aligned bounding box for the point cloud
bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(pcd6.points)
x_min = bbox.get_min_bound()[0]
x_max = bbox.get_max_bound()[0]
width = x_max - x_min
print("Width of the point cloud in the x-direction:", width)

distance = pcd1.compute_point_cloud_distance(pcd1)
print(np.mean(distance))


# # Create a line geometry with two points
# line_points = [[x_min, 0, 0], [x_max, 0, 0]]
# line_colors = [[1, 0, 0] for i in range(2)]
# line_set = o3d.geometry.LineSet(
#     points=o3d.utility.Vector3dVector(line_points),
#     lines=o3d.utility.Vector2iVector([[0, 1]]))
# line_set.colors = o3d.utility.Vector3dVector(line_colors)

# Create a line set from the point cloud
points = [[x_min, 0, 0], [x_max, 0, 0]]
lines = [[0, 1]]
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(points), lines=o3d.utility.Vector2iVector(lines))

# # Calculate the Euclidean distance between two points
# def euclidean_distance(p1, p2):
#     return np.sqrt(np.sum((p1 - p2) ** 2))
#
# # Calculate the distance between pcd and pcd2
# distance = euclidean_distance(pcd1_position, pcd2_position)

# # Print the distance
# print("Distance between pcd and pcd2:", distance)

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd1, pcd2, pcd3, pcd4, pcd5, pcd6, line_set])

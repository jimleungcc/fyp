import open3d as o3d
import numpy as np

# Load two point clouds
pcd1 = o3d.io.read_point_cloud("door.ply")
pcd2 = o3d.io.read_point_cloud("Area_5/table.ply")
pcd3 = o3d.io.read_point_cloud("Area_5/westchair4.ply")
pcd4 = o3d.io.read_point_cloud("Area_5/eastchair2.ply")
pcd5 = o3d.io.read_point_cloud("Area_5/westchair3.ply")

# Adjust position of object
pcd1.translate([10,0.0,0.0])
# rotation = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
# pcd1.rotate(rotation)

# Create bounding boxes for point clouds
bbox1 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(pcd1.points)
bbox2 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(pcd2.points)
bbox3 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(pcd3.points)
bbox4 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(pcd4.points)
bbox5 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(pcd5.points)

# Set colors for bounding boxes
bbox1.color = [1, 0, 0] # red
bbox2.color = [0, 1, 0] # green


# Compute the distances between all pairs of vertices
vertices1 = np.asarray(bbox1.get_box_points())
vertices2 = np.asarray(bbox2.get_box_points())
distances = np.linalg.norm(vertices1[:, None, :] - vertices2[None, :, :], axis=-1)

# Find the two closest vertices and compute the shortest distance
min_distance_indices = np.unravel_index(np.argmin(distances), distances.shape)
shortest_distance = distances[min_distance_indices]

# Create a LineSet object for the line segment
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector([vertices1[min_distance_indices[0]], vertices2[min_distance_indices[1]]])
line_set.lines = o3d.utility.Vector2iVector([[0, 1]])
line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0]])

# Visualize point clouds, bounding boxes, and the line segment
geometries = [pcd1, pcd2, bbox1, bbox2, pcd3, pcd4, pcd5]
if line_set is not None:
    geometries.append(line_set)

center1 = bbox1.get_center()
center2 = bbox2.get_center()

# Print the shortest distance
print("Shortest distance between the two bounding boxes:", shortest_distance)
o3d.visualization.draw_geometries(geometries)

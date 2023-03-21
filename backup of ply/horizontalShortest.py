import open3d as o3d
import numpy as np

# Load two point clouds
pcd1 = o3d.io.read_point_cloud("door.ply")
pcd2 = o3d.io.read_point_cloud("table.ply")

# Create bounding boxes for the point clouds
bbox1 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(pcd1.points)
bbox2 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(pcd2.points)

# Get the x-coordinates of the bounding boxes
x_min1 = bbox1.get_min_bound()[0]
x_max1 = bbox1.get_max_bound()[0]
x_min2 = bbox2.get_min_bound()[0]
x_max2 = bbox2.get_max_bound()[0]

# Calculate the horizontal shortest distance between the bounding boxes
if x_max1 < x_min2:
    dist = x_min2 - x_max1
    line_start = np.array([x_max1, 0, 0])
    line_end = np.array([x_min2, 0, 0])
elif x_max2 < x_min1:
    dist = x_min1 - x_max2
    line_start = np.array([x_max2, 0, 0])
    line_end = np.array([x_min1, 0, 0])
else:
    dist = 0
    line_start = np.array([0, 0, 0])
    line_end = np.array([0, 0, 0])

# Print the distance
print("Horizontal shortest distance:", dist)

# Visualize the distance with a line
line = o3d.geometry.LineSet()
line.points = o3d.utility.Vector3dVector([line_start, line_end])
line.lines = o3d.utility.Vector2iVector([[0, 1]])
line.paint_uniform_color([1, 0, 0]) # red color
o3d.visualization.draw_geometries([pcd1, pcd2, line])

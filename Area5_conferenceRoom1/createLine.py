import open3d as o3d
import numpy as np

# Define two points
p1 = np.array([0, 0, 0])
p2 = np.array([1, 1, 1])

pcd = o3d.io.read_point_cloud("door.ply")

# Create a line with 10 points
# line_points = np.linspace(p1, p2, num=10)
# line_set = o3d.geometry.LineSet()
# line_set.points = o3d.utility.Vector3dVector([p1, p2])
# line_set.lines = o3d.utility.Vector2iVector(np.array([[0, 1]]))
# print(line_points)

line_points = np.linspace(p1, p2, num=10)
line_pcd = o3d.geometry.PointCloud()
line_pcd.points = o3d.utility.Vector3dVector(line_points)

dist = line_pcd.compute_point_cloud_distance(pcd)
print(len(dist))
print(len(pcd.points))

# Visualize the line
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(line_pcd)
vis.run()
vis.destroy_window()

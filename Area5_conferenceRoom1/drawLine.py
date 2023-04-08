import open3d as o3d
import numpy as np

# Load two point clouds
pcd1 = o3d.io.read_point_cloud("Area_5/westchair1.ply")
pcd2 = o3d.io.read_point_cloud("Area_5/westwall_combine.ply")

# Compute point cloud distances
distances = pcd1.compute_point_cloud_distance(pcd2)

# Get indices of nearest neighbors
indices = np.argmin(distances, axis=0)
if np.isscalar(indices):
    indices = np.array([indices])
print(f"indices: {indices}")

# Get corresponding points
points1 = np.asarray(pcd1.points)
points2 = np.asarray(pcd2.points)
correspondences = np.hstack((np.arange(len(indices)).reshape(-1, 1), indices.reshape(-1, 1)))
print(f"correspondences: {correspondences}")

# Create line set from correspondences
line_set = o3d.geometry.LineSet.create_from_point_cloud_correspondences(
    pcd1, pcd2, correspondences)

# Visualize the line set
o3d.visualization.draw_geometries([pcd1, pcd2, line_set])
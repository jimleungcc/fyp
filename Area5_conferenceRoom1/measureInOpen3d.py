import open3d as o3d

# Load a PLY file
pcd = o3d.io.read_point_cloud("northwall_combine.ply")

# Get the AABB of the point cloud
aabb = pcd.get_axis_aligned_bounding_box()

# Get the width of the AABB
width = aabb.get_max_bound()[0] - aabb.get_min_bound()[0]

print("Width of the point cloud: ", width)

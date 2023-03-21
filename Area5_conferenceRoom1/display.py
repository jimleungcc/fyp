import open3d as o3d
import numpy as np

print(o3d.__version__)

# Load the PLY obstacle files.
obstacles = []
obstacle1 = o3d.io.read_point_cloud("/home/jim/Desktop/softgroup/PLYs/Area5_conferenceRoom1/obj28#ff7f00.ply")
obstacle2 = o3d.io.read_point_cloud("/home/jim/Desktop/softgroup/PLYs/Area5_conferenceRoom1/obj2#00547f.ply")
obstacles.append(obstacle1)
obstacles.append(obstacle2)

# Load the starting and destination points.
start_point = o3d.io.read_point_cloud("/home/jim/Desktop/softgroup/PLYs/Area5_conferenceRoom1/obj4#00aa7f.ply")
destination_point = o3d.io.read_point_cloud("/home/jim/Desktop/softgroup/PLYs/Area5_conferenceRoom1/obj12#54aa7f.ply")

# Create a point cloud and add the starting and destination points to it.
pointcloud = o3d.geometry.PointCloud()
pointcloud += start_point
pointcloud += destination_point

# Register the obstacles to the point cloud.
for obstacle in obstacles:
    T, _ = registration.registration_icp(obstacle, pointcloud, 1.0, np.eye(4), 
                                              registration.TransformationEstimationPointToPoint())
    obstacle.transform(T)

# Create an AxisAlignedBoundingBox for each obstacle.
boxes = []
for obstacle in obstacles:
    bbox = obstacle.get_axis_aligned_bounding_box()
    boxes.append(bbox)

# Create a VoxelGrid for the obstacles.
voxel_size = 0.1
distance_voxel_grid = o3d.geometry.VoxelGrid.create_distance_voxel_grid_from_point_cloud(obstacle1, voxel_size)

# Convert each AxisAlignedBoundingBox to a VoxelGrid.
signed_distance_fields = []
for box in boxes:
    signed_distance_field = box.create_signed_distance_voxel_grid(voxel_size, 5)
    signed_distance_fields.append(signed_distance_field)

# Create a SignedDistanceField for the point cloud.
sdf = o3d.geometry.SignedDistanceField.create_from_point_cloud(pointcloud, voxel_size)

# Find the shortest path while avoiding the obstacles.
path = o3d.geometry.path.shortest_path_point_cloud_sdf(sdf, start_point, destination_point, 
                                                       signed_distance_fields, voxel_size)

# Visualize the point cloud, obstacles, and shortest path.
vis = o3d.visualization.Visualizer()
vis.create_window()

# Add the point cloud, obstacles, and shortest path to the visualizer window.
vis.add_geometry(pointcloud)
for obstacle in obstacles:
    vis.add_geometry(obstacle)
vis.add_geometry(path)

# Add a coordinate system to the scene.
coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
vis.add_geometry(coord_frame)

# Run the visualizer.
vis.run()
vis.destroy_window()
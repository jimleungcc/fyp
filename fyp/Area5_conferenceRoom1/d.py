import open3d as o3d
import numpy as np

obstacle = o3d.io.read_point_cloud("/home/jim/Desktop/softgroup/PLYs/Area5_conferenceRoom1/obj28#ff7f00.ply")
# Load destination point cloud
destination = o3d.io.read_point_cloud("/home/jim/Desktop/softgroup/PLYs/Area5_conferenceRoom1/obj2#00547f.ply")


# Register obstacle to point cloud
T, _ = o3d.registration.registration_icp(obstacle, pointcloud, 1.0, np.eye(4), 
                                           o3d.registration.TransformationEstimationPointToPoint(),
                                           o3d.registration.ICPConvergenceCriteria(max_iteration=1000))

# Apply transformation to obstacle
obstacle.transform(T)

# Define start and end points
start_point = [0.0, 0.0, 0.0]
destination_point = [1.0, 1.0, 1.0]

# Define the threshold for collision detection
collision_threshold = 0.01

# Create an empty point cloud to store the path
path_pointcloud = o3d.geometry.PointCloud()

# Initialize the path with the start point
path_pointcloud.points.append(o3d.utility.Vector3d(start_point))

# Find the path from start to destination point
while True:
    # Get the last point in the path
    last_point = path_pointcloud.points[-1]

    # Compute the direction vector from the last point to the destination point
    direction = np.array(destination_point) - np.array(last_point)

    # Normalize the direction vector
    direction /= np.linalg.norm(direction)

    # Compute the next point in the path
    next_point = last_point + direction * 0.1

    # Check for collision with obstacle
    if obstacle.compute_point_cloud_distance(o3d.geometry.PointCloud(o3d.utility.Vector3dVector([next_point])))[1][0] > collision_threshold and \
       path_pointcloud.compute_point_cloud_distance(o3d.geometry.PointCloud(o3d.utility.Vector3dVector([next_point])))[1][0] > 0.01:
        # No collision detected, add the next point to the path
        path_pointcloud.points.append(o3d.utility.Vector3d(next_point))
    else:
        # Collision detected, stop the path finding
        break

# Visualize the path and point cloud
vis = o3d.visualization.Visualizer()
vis.create_window()

# Add point cloud and obstacle to the visualizer
vis.add_geometry(pointcloud)
vis.add_geometry(obstacle)

# Set the path color to red
path_pointcloud.paint_uniform_color([1, 0, 0])

# Add the path to the visualizer
vis.add_geometry(path_pointcloud)

# Run the visualizer
vis.run()
vis.destroy_window()




import os
import open3d as o3d
import numpy as np

# Load PLY file
pcd = o3d.io.read_point_cloud("Room2.ply")

# Get point colors as numpy array
colors = np.asarray(pcd.colors)

# Get unique colors in the point cloud
unique_colors = np.unique(colors, axis=0)

# Create a directory to store the output files
dirname = os.path.splitext(os.path.basename("Room2.ply"))[0] # Extract the file name without extension
os.makedirs(dirname, exist_ok=True)

# Separate points by color and create new point clouds for each color
for idx, color in enumerate(unique_colors):
    # Get the indices of the points with this color
    indices = np.where((colors == color).all(axis=1))[0]

    # Create new point cloud with points of this color
    color_pcd = pcd.select_by_index(indices)

    # Convert color to Hex format
    color_hex = "#{:02x}{:02x}{:02x}".format(int(color[0]*255), int(color[1]*255), int(color[2]*255))

    # Set filename based on object number and color in Hex format
    filename = os.path.join(dirname, "obj{}{}.ply".format(idx, color_hex))

    # Write point cloud to file
    o3d.io.write_point_cloud(filename, color_pcd)

# import numpy as npand westside
# import plyfile
#
# # Load the ply file
# plydata = plyfile.PlyData.read('eastwall_combine.ply')
# # plydata = plyfile.PlyData.read('table.ply')
#
#
# # Extract the vertex coordinates
# vertices = np.array([(x, y, z) for x, y, z in zip(plydata['vertex']['x'],
#                                                   plydata['vertex']['y'],
#                                                   plydata['vertex']['z'])])
#
# # Compute the bounding box
# xmin, ymin, zmin = np.min(vertices, axis=0)
# xmax, ymax, zmax = np.max(vertices, axis=0)
#
# # Compute the width of the bounding box
# width = xmax - xmin
#
# print('Bounding box width:', width)

import numpy as np
import plyfile

# Load the ply file
plydata = plyfile.PlyData.read('northwall_combine.ply')

# Extract the vertex coordinates
vertices = np.array([(x, y, z) for x, y, z in zip(plydata['vertex']['x'],
                                                  plydata['vertex']['y'],
                                                  plydata['vertex']['z'])])

# Compute the bounding box
xmin, ymin, zmin = np.min(vertices, axis=0)
xmax, ymax, zmax = np.max(vertices, axis=0)

# Compute the dimensions of the bounding box
width = xmax - xmin
height = ymax - ymin
length = zmax - zmin

print('Bounding box dimensions (width, height, length):', width, height, length)

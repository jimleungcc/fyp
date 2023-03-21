import open3d as o3d
import numpy as np

# load all chair PLYs in the west
westchair1 = o3d.io.read_point_cloud("westchair1.ply")
westchair2 = o3d.io.read_point_cloud("westchair2.ply")
westchair3 = o3d.io.read_point_cloud("westchair3.ply")
westchair4 = o3d.io.read_point_cloud("westchair4.ply")

# load all chair PLYs in the east
eastchair1 = o3d.io.read_point_cloud("eastchair1.ply")
eastchair2 = o3d.io.read_point_cloud("eastchair2.ply")
eastchair3 = o3d.io.read_point_cloud("eastchair3.ply")

# load table, floor and door
table = o3d.io.read_point_cloud("table.ply")
# door = o3d.io.read_point_cloud("door.ply")
floor = o3d.io.read_point_cloud("floor.ply")

# load all wall PLYs
northwall = o3d.io.read_point_cloud("northwall.ply")
eastwall = o3d.io.read_point_cloud("eastwall1.ply")
southwall = o3d.io.read_point_cloud("southwall.ply")
westwall = o3d.io.read_point_cloud("westwall1.ply")

output = o3d.io.read_point_cloud("output_file.ply")

objectList = []
objectList.append(output)

objectList.append(table)
# objectList.append(door)
objectList.append(floor)
objectList.append(northwall)
objectList.append(eastwall)
objectList.append(southwall)
objectList.append(westwall)

o3d.visualization.draw_geometries(objectList)

# hello world

import numpy
import open3d as o3d

westchair1 = o3d.io.read_point_cloud("Area_5/westchair1.ply")
westwall = o3d.io.read_point_cloud("Area_5/westwall_combine.ply")

dist = westchair1.compute_point_cloud_distance(westwall)
print("numpy min:", numpy.min(dist))
print("numpy max:", numpy.max(dist))
print("numpy avg:", numpy.mean(dist))
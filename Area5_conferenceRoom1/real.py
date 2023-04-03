import numpy as np
import open3d as o3d
import open3d.utility
from plyfile import PlyData, PlyElement

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
door = o3d.io.read_point_cloud("door.ply")
floor = o3d.io.read_point_cloud("floor.ply")

# load all wall PLYs
northwall = o3d.io.read_point_cloud("northwall_combine.ply")
eastwall = o3d.io.read_point_cloud("eastwall_combine.ply")
southwall = o3d.io.read_point_cloud("southwall.ply")
westwall = o3d.io.read_point_cloud("westwall_combine.ply")

# distances = westwall.compute_point_cloud_distance(westchair1)
# distance = sum(distances)/len(distances)
# print("Distance between the two point clouds:", distance)

westSide = []
westSide.append(westchair1)
westSide.append(westchair2)
westSide.append(westchair3)
westSide.append(westchair4)
westSide.append(table)

eastSide = []
eastSide.append(eastchair1)
eastSide.append(eastchair2)
eastSide.append(eastchair3)
eastSide.append(table)

# dist = westchair1.compute_point_cloud_distance(westwall)
# print("numpy min:", numpy.min(dist))
# print("numpy max:", numpy.max(dist))
# print("numpy avg:", numpy.mean(dist))


# Assume the object width is same as the door
# Get the axis_aligned_bounding_box of the point cloud
aabb = door.get_axis_aligned_bounding_box()
# Get the width of the AABB
width = (aabb.get_max_bound()[0] - aabb.get_min_bound()[0])
# print("Width of the door: ", width)
height = (aabb.get_max_bound()[2] - aabb.get_min_bound()[2])
# print("Height of the door: ", height)
length = (aabb.get_max_bound()[1] - aabb.get_min_bound()[1])
# print("Length of the door: ", length)
center = aabb.get_center()

destination_box = northwall.get_axis_aligned_bounding_box()
destination_center = destination_box.get_center()

width = 1
possiblePath = 0
west_stop_list = []
east_stop_list = []
stop_list = []
objectList = []

left = []
middle = []
right = []


def get_section():
    # Get the 1/3 width from the floor

    floor_box = floor.get_minimal_oriented_bounding_box()
    floor_width = (floor_box.get_max_bound()[0] - floor_box.get_min_bound()[0])
    print("\nfloor width: {}".format(floor_width))
    floor_length = (floor_box.get_max_bound()[1] - floor_box.get_min_bound()[1])
    print("floor length: {}".format(floor_length))
    floor_height = (floor_box.get_max_bound()[2] - floor_box.get_min_bound()[2])
    print("floor height: {}".format(floor_height))

    floorbox_8points = np.asarray(floor_box.get_box_points())
    np.set_printoptions(suppress=True, precision=15)
    # print("Before sort", floorbox_8points)

    sorted_idx = np.argsort(floorbox_8points[:, 0])
    sorted_a = floorbox_8points[sorted_idx]
    # print("floor 8 points: ", sorted_a)
    # objectList.append(floor_box)

    # p2 = o3d.io.read_point_cloud("point2.ply")
    # objectList.append(p2)

    # p3 = o3d.io.read_point_cloud("sortedAllPoint.ply")
    # objectList.append(p3)

    door_box = door.get_minimal_oriented_bounding_box()
    door_8points = np.asarray(door_box.get_box_points())
    # print("door 8 points", door_8points)
    sorted_idx = np.argsort(door_8points[:, 0])
    sorted_a = door_8points[sorted_idx]
    print("sorted door 8 point", sorted_a)


    p4 = o3d.io.read_point_cloud("doorallpoint.ply")
    objectList.append(p4)


    # get the min x and max x coordinates
    min_x = floor_box.get_min_bound()[0]
    max_x = floor_box.get_max_bound()[0]
    print("min x {}".format(min_x))
    print("max x {}".format(max_x))
    diff = max_x - min_x
    third = diff / 3
    print("diff: {}".format(diff))
    # print("1/3 of diff: {}".format(third))
    print("1/3: {}".format(min_x + third))
    print("2/3: {}".format(min_x + third * 2))

    min_y = floor_box.get_min_bound()[1]
    max_y = floor_box.get_max_bound()[1]
    print("\n")
    print("min y {}".format(min_y))
    print("max y {}".format(max_y))
    diff = max_y - min_y
    third = diff / 3
    print("diff: {}".format(diff))
    # print("1/3 of diff: {}".format(third))
    print("1/3: {}".format(max_y - third))
    print("2/3: {}".format(max_y - third - third))

    print("\nby me")
    floor_max_x = floor.get_max_bound()[0]
    floor_min_x = floor.get_min_bound()[0]
    floor_max_y = floor.get_max_bound()[1]
    floor_min_y = floor.get_min_bound()[1]
    floor_max_z = floor.get_max_bound()[2]
    floor_min_z = floor.get_min_bound()[2]
    print(floor_max_x)
    print(floor_min_x)
    print(floor_max_y)
    print(floor_min_y)
    print(floor_max_z)
    print(floor_min_z)

    diff = floor_max_y - floor_min_y
    floor_third = diff / 3
    print("diff: {}".format(diff))
    print("floor_third: {}".format(floor_third))
    print("my 1/3: {}".format(floor_max_y - floor_third))
    print("my 2/3: {}".format(floor_max_y - floor_third - floor_third))

    # line_set = o3d.geometry.LineSet()
    # line_set.points = o3d.utility.Vector3dVector(
    #     np.array([[-6.455, -15.501, 0.0445], [-0.998, -15.501, 0.0445]]))
    # line_set.lines = o3d.utility.Vector2iVector(np.array([[0, 1]]))
    # objectList.append(line_set)

    # pt = o3d.io.read_point_cloud("point1.ply")
    # objectList.append(pt)


get_section()


def return_coordinates(obstacle, distance_array):
    obstacle_np = np.asarray(obstacle.points)
    closest_index = np.argmin(distance_array, axis=0)
    closest_coordinates_from_object = obstacle_np[closest_index]
    return closest_coordinates_from_object


def draw_line(coordinate_list):
    print("coord_list length: {}".format(len(coordinate_list)))
    print(coordinate_list)
    for i in range(len(coordinate_list)):
        if i != len(coordinate_list) - 1:
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(np.array([coordinate_list[i], coordinate_list[i + 1]]))
            line_set.lines = o3d.utility.Vector2iVector(np.array([[0, 1]]))
            objectList.append(line_set)


def check_enough_space(side, wall, objList):
    print("\nTest {}:".format(side))
    print("{} object list length: {}".format(side, len(objList)))
    stop_list.clear()
    stop_list.append(np.array(center))
    for i in range(len(objList)):
        print("i in check: {}".format(i))
        if i == len(objList) - 1:
            distance = objList[i].compute_point_cloud_distance(northwall)
            min_distance = np.min(distance)
        else:
            distance = objList[i].compute_point_cloud_distance(wall)
            min_distance = np.min(distance)
        print("obj{} minDistance in {}: {}".format(i, side, min_distance))

        if min_distance < width:
            print("Obj{} not enough width in {}".format(i, side))
            print("{} cannot pass".format(side))
            return 0
        else:
            stop_list.append(return_coordinates(objList[i], distance))
    # Add the destination point
    stop_list.append(destination_center)
    draw_line(stop_list)
    return 1


possiblePath += check_enough_space("east", eastwall, eastSide)
possiblePath += check_enough_space("west", westwall, westSide)

print("\npossible path:", possiblePath)

objectList.append(table)
objectList.append(door)
objectList.append(floor)

# objectList.append(northwall)
# objectList.append(eastwall)
# objectList.append(southwall)
# objectList.append(westwall)
#
# objectList.append(westchair1)
# objectList.append(westchair2)
# objectList.append(westchair3)
# objectList.append(westchair4)

# objectList.append(eastchair1)
# objectList.append(eastchair2)
objectList.append(eastchair3)

# points = np.array([[0.1, 0.1, 0.1], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
# colors = [[1, 1, 1], [1, 0, 0], [0, 1, 0], [0, 0, 1]]
# test_pcd = open3d.geometry.PointCloud()
# test_pcd.points = open3d.utility.Vector3dVector(points)
# test_pcd.colors = open3d.utility.Vector3dVector(colors)
# objectList.append(test_pcd)

o3d.visualization.draw_geometries(objectList)

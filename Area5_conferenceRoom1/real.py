import numpy as np
import open3d as o3d
import open3d.utility
import math
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


def get_rectangle_area(x1, y1, x2, y2, x3, y3, x4, y4):
    # Length of the base
    base_length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # Height of the rectangle
    height = math.sqrt((x3 - x2) ** 2 + (y3 - y2) ** 2)

    # Area of the rectangle
    area = base_length * height

    print("The area of the rectangle is:", area, "square units.")

    return area


def inside_rectangle(point1, point2, point3, point4, px, py, rectangle_area):
    x1, y1 = point1[0], point1[1]
    x2, y2 = point2[0], point2[1]
    x3, y3 = point3[0], point3[1]
    x4, y4 = point4[0], point4[1]

    # print("x1, y1:", x1, y1)
    # print("x2, y2:", x2, y2)
    # print("x3, y3:", x3, y3)
    # print("x4, y4:", x4, y4)
    # print("px, py", px, py)

    def triangle_area(px, py, x1, y1, x2, y2):
        area = 0.5 * abs((x1 * (y2 - py) + x2 * (py - y1) + px * (y1 - y2)))
        return area

    t1 = triangle_area(px, py, x1, y1, x2, y2)
    t2 = triangle_area(px, py, x2, y2, x3, y3)
    t3 = triangle_area(px, py, x3, y3, x4, y4)
    t4 = triangle_area(px, py, x1, y1, x4, y4)
    area_sum_of_triangle = t1 + t2 + t3 + t4

    round_rect_area = round(rectangle_area, 5)
    round_total_triangles_area = round(area_sum_of_triangle, 5)

    if round_rect_area == round_total_triangles_area:
        return 1
    else:
        return 0


def find_2_third_coordinates(x1, y1, x2, y2):
    # Define the endpoints of the line segment as tuples
    p1 = (x1, y1)
    p2 = (x2, y2)

    # Calculate the length of the line segment
    d = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    # Calculate the distance from the first point to the first 1/3 point
    d_1_3 = (1 / 3) * d

    # Calculate the slope of the line segment
    m = (p2[1] - p1[1]) / (p2[0] - p1[0])

    # Calculate the x-coordinate of the first 1/3 point
    x_1_3_1 = p1[0] + d_1_3 / math.sqrt(1 + m ** 2)

    # Calculate the y-coordinate of the first 1/3 point
    y_1_3_1 = p1[1] + m * (d_1_3 / math.sqrt(1 + m ** 2))

    # Calculate the distance from the first point to the second 1/3 point
    d_2_3 = (2 / 3) * d

    # Calculate the x-coordinate of the second 1/3 point
    x_1_3_2 = p1[0] + d_2_3 / math.sqrt(1 + m ** 2)

    # Calculate the y-coordinate of the second 1/3 point
    y_1_3_2 = p1[1] + m * (d_2_3 / math.sqrt(1 + m ** 2))

    # Return the coordinates of both 1/3 points as a tuple
    return ((x_1_3_1, y_1_3_1), (x_1_3_2, y_1_3_2))


def get_section():
    # Get the 1/3 width from the floor

    floor_box = floor.get_minimal_oriented_bounding_box()
    floor_width = (floor_box.get_max_bound()[0] - floor_box.get_min_bound()[0])
    # print("\nfloor width: {}".format(floor_width))
    floor_length = (floor_box.get_max_bound()[1] - floor_box.get_min_bound()[1])
    # print("floor length: {}".format(floor_length))
    floor_height = (floor_box.get_max_bound()[2] - floor_box.get_min_bound()[2])
    # print("floor height: {}".format(floor_height))

    floorbox_8points = np.asarray(floor_box.get_box_points())
    np.set_printoptions(suppress=True, precision=15)

    sorted_idx = np.argsort(floorbox_8points[:, 0])
    sorted_a = floorbox_8points[sorted_idx]
    print("sorted floor 8 points: ", sorted_a)
    print("\n")
    rect_points = []
    rect_points.append(sorted_a[6])
    rect_points.append(sorted_a[4])
    rect_points.append(sorted_a[0])
    rect_points.append(sorted_a[2])
    # 7 > 5 > 1 > 3
    print("rect_points\n", rect_points)

    # Get the 1/3 coordinate of the side
    line1_first_third, line1_second_third = find_2_third_coordinates(rect_points[1][0], rect_points[1][1],
                                                                     rect_points[0][0], rect_points[0][1])
    print("line1 1/3", line1_first_third)
    print("line1 2/3", line1_second_third)

    line2_first_third, line2_second_third = find_2_third_coordinates(rect_points[2][0], rect_points[2][1],
                                                                     rect_points[3][0], rect_points[3][1])
    print("\nline2 1/3", line2_first_third)
    print("line2 2/3", line2_second_third)

    # left section
    left_rectangle_area = get_rectangle_area(rect_points[0][0], rect_points[0][1],
                                             line1_second_third[0], line1_second_third[1],
                                             line2_second_third[0], line2_second_third[1],
                                             rect_points[3][0], rect_points[3][1])
    print("left rectangle area", left_rectangle_area)

    point_from_westchair2 = westchair2.points
    print("chair vertex length", len(np.asarray(point_from_westchair2)))

    print("\n")
    print(rect_points[0])
    print(line1_second_third)
    print(line2_second_third)
    print(rect_points[3])
    print(point_from_westchair2[0][0])
    print(point_from_westchair2[0][1])

    left_section = 0
    for i in range(len(np.asarray(point_from_westchair2))):
        current_x = point_from_westchair2[i][0]
        current_y = point_from_westchair2[i][1]
        left_section += inside_rectangle(rect_points[0], line1_second_third, line2_second_third, rect_points[3],
                                         current_x, current_y, left_rectangle_area)

    print("vertex number in left", left_section)


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


#
# possiblePath += check_enough_space("east", eastwall, eastSide)
# possiblePath += check_enough_space("west", westwall, westSide)
#
# print("\npossible path:", possiblePath)

objectList.append(table)
objectList.append(door)
objectList.append(floor)

# objectList.append(northwall)
objectList.append(eastwall)
objectList.append(southwall)
objectList.append(westwall)

objectList.append(westchair1)
objectList.append(westchair2)
objectList.append(westchair3)
objectList.append(westchair4)

objectList.append(eastchair1)
objectList.append(eastchair2)
objectList.append(eastchair3)

# The coordinate system
points = np.array([[0.1, 0.1, 0.1], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
colors = [[1, 1, 1], [1, 0, 0], [0, 1, 0], [0, 0, 1]]
test_pcd = open3d.geometry.PointCloud()
test_pcd.points = open3d.utility.Vector3dVector(points)
test_pcd.colors = open3d.utility.Vector3dVector(colors)
objectList.append(test_pcd)

o3d.visualization.draw_geometries(objectList)

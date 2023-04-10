import numpy as np
import open3d as o3d
import open3d.utility
import math
from tree import TreeNode
import time

st = time.time()
"""
    load all PLY files in here
"""
# load all chair PLYs in the west
westchair1 = o3d.io.read_point_cloud("Area5_conferenceRoom1/Area_5/westchair1.ply")
westchair2 = o3d.io.read_point_cloud("Area5_conferenceRoom1/Area_5/westchair2.ply")
westchair3 = o3d.io.read_point_cloud("Area5_conferenceRoom1/Area_5/westchair3.ply")
westchair4 = o3d.io.read_point_cloud("Area5_conferenceRoom1/Area_5/westchair4.ply")

# load all chair PLYs in the east
eastchair1 = o3d.io.read_point_cloud("Area5_conferenceRoom1/Area_5/eastchair1.ply")
eastchair2 = o3d.io.read_point_cloud("Area5_conferenceRoom1/Area_5/eastchair2.ply")
eastchair3 = o3d.io.read_point_cloud("Area5_conferenceRoom1/Area_5/eastchair3.ply")

# load table, floor and door
table = o3d.io.read_point_cloud("Area5_conferenceRoom1/Area_5/table.ply")
door = o3d.io.read_point_cloud("Area5_conferenceRoom1/door.ply")
floor = o3d.io.read_point_cloud("Area5_conferenceRoom1/Area_5/floor.ply")

# load all wall PLYs
northwall = o3d.io.read_point_cloud("Area5_conferenceRoom1/Area_5/northwall_combine.ply")
eastwall = [o3d.io.read_point_cloud("Area5_conferenceRoom1/Area_5/eastwall_combine.ply"), "eastwall"]
southwall = o3d.io.read_point_cloud("Area5_conferenceRoom1/Area_5/southwall.ply")
westwall = [o3d.io.read_point_cloud("Area5_conferenceRoom1/Area_5/westwall_combine.ply"), "westwall"]
"""
    End: load all PLY files
"""

"""
    Append all obstacles to the obstacles_list in here, 
    walls are not considered as obstacles.
    The data structure must be [loaded point cloud, "name of the point cloud"]
"""
obstacles_list = []
obstacles_list.append([table, "table"])
obstacles_list.append([westchair1, "westchair1"])
obstacles_list.append([eastchair1, "eastchair1"])
obstacles_list.append([westchair2, "westchair2"])
obstacles_list.append([eastchair2, "eastchair2"])
obstacles_list.append([westchair3, "westchair3"])
obstacles_list.append([eastchair3, "eastchair3"])
obstacles_list.append([westchair4, "westchair4"])
"""
    End: Append all obstacles to the obstacles_list in here
"""

"""
    Perform voxel down sampling here, comment the codes between if necessary
"""
eastwall[0] = eastwall[0].voxel_down_sample(voxel_size=0.05)
westwall[0] = westwall[0].voxel_down_sample(voxel_size=0.05)

for i in range(len(obstacles_list)):
    obstacles_list[i][0] = obstacles_list[i][0].voxel_down_sample(voxel_size=0.05)
"""
    End: Voxel down sampling
"""


"""
    This part use the door width to determine as a reference to determine the user define width,
    can be ignored.
"""
# Use the door width to determine as a reference to determine the user define width
# Get the axis_aligned_bounding_box of the point cloud
aabb = door.get_axis_aligned_bounding_box()
# Get the width of the AABB
door_width = (aabb.get_max_bound()[0] - aabb.get_min_bound()[0])
# print("Width of the door: ", door_width)
door_height = (aabb.get_max_bound()[2] - aabb.get_min_bound()[2])
# print("Height of the door: ", door_height)
door_length = (aabb.get_max_bound()[1] - aabb.get_min_bound()[1])
# print("Length of the door: ", door_length)
center = aabb.get_center()
print("start center coord:", center)

destination_box = northwall.get_axis_aligned_bounding_box()
destination_center = destination_box.get_center()
print("destination center:", destination_center)

door_width = door_width / 2  # half width of the door
print("door width/2:", door_width)  # 0.4681645000000003
"""
    End: reference number
"""


"""
    ***************** User defined width *****************
    modify the value to see different path finding results
"""
width = 0.22
"""
    ***************** End: User defined width *****************
"""


"""
    DO NOT DELETE CODES BETWEEN
"""
display_list = []
left_obj_list = []
middle_obj_list = []
right_obj_list = []
"""
    End: DO NOT DELETE CODES BETWEEN
"""


def get_rectangle_area(point1, point2, point3, point4):
    # Length of the base
    x1, y1 = point1[0], point1[1]
    x2, y2 = point2[0], point2[1]
    x3, y3 = point3[0], point3[1]
    x4, y4 = point4[0], point4[1]

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
    # floor_width = (floor_box.get_max_bound()[0] - floor_box.get_min_bound()[0])
    # floor_length = (floor_box.get_max_bound()[1] - floor_box.get_min_bound()[1])
    # floor_height = (floor_box.get_max_bound()[2] - floor_box.get_min_bound()[2])

    floorbox_8points = np.asarray(floor_box.get_box_points())
    np.set_printoptions(suppress=True, precision=15)

    sorted_idx = np.argsort(floorbox_8points[:, 0])
    sorted_a = floorbox_8points[sorted_idx]

    """
        ************* The array indexs represent the 8 vertices of the floor's bounding box,
                      it is used for define the rectangle, as the behaviour of the 8 vertices order is a little bit
                      randomized in Open3D, therefore, sometimes it is necessary examine all 8 points and 
                      decide which 4 vertices should be use as the rectangle vertices. 
                      To do so: 
                      1. uncomment print("sorted floor 8 points: ", sorted_a)
                      2. Copy the printed 8 vertex and create a PLY file to store it
                      3. Change the RGB value to identify the specific vertex
                      4. Use MeshLab to view the PLY file
                      5. Figure the 4 vertices of the floor
                      6. Record the index of the 4 vertices
                      7. Modify the array index of these 4 lines:
                         rect_points.append(sorted_a[6]) -> P1
                         rect_points.append(sorted_a[4]) -> P2
                         rect_points.append(sorted_a[0]) -> P3
                         rect_points.append(sorted_a[2]) -> P4
                         P1                         P2
                         +---------------------------+
                         |                           |
                         +---------------------------+
                         P4                         P3
                         The order should be either clockwise or anti-clockwise, otherwise the program cannot find
                         the correct dimension info of the floor.
    """
    # print("sorted floor 8 points: ", sorted_a)
    # print("\n")
    rect_points = []
    rect_points.append(sorted_a[6])
    rect_points.append(sorted_a[4])
    rect_points.append(sorted_a[0])
    rect_points.append(sorted_a[2])
    # 7 > 5 > 1 > 3
    print("rect_points\n", rect_points)
    """
        End of Caution
    """

    # Get the 1/3 coordinate of the side
    line1_first_third, line1_second_third = find_2_third_coordinates(rect_points[1][0], rect_points[1][1],
                                                                     rect_points[0][0], rect_points[0][1])
    print("\nline1 1/3", line1_first_third)
    print("line1 2/3", line1_second_third)

    line2_first_third, line2_second_third = find_2_third_coordinates(rect_points[2][0], rect_points[2][1],
                                                                     rect_points[3][0], rect_points[3][1])
    print("\nline2 1/3", line2_first_third)
    print("line2 2/3", line2_second_third)

    # left section
    left_rectangle_area = get_rectangle_area(rect_points[0], line1_second_third, line2_second_third, rect_points[3])
    print("left rectangle area", left_rectangle_area)

    middle_rectangle_area = get_rectangle_area(line1_second_third, line1_first_third,
                                               line2_first_third, line2_second_third)
    print("middle rectangle area", middle_rectangle_area)

    right_rectangle_area = get_rectangle_area(line1_first_third, rect_points[1], rect_points[2], line2_first_third)
    print("right rectangle area", right_rectangle_area)

    for obj in obstacles_list:
        left_section = 0
        middle_section = 0
        right_section = 0
        point_from_object = obj[0].points
        object_vertex_length = len(np.asarray(point_from_object))
        for i in range(object_vertex_length):
            current_x = point_from_object[i][0]
            current_y = point_from_object[i][1]
            left_section += inside_rectangle(rect_points[0], line1_second_third, line2_second_third, rect_points[3],
                                             current_x, current_y, left_rectangle_area)
            middle_section += inside_rectangle(line1_second_third, line1_first_third,
                                               line2_first_third, line2_second_third,
                                               current_x, current_y, middle_rectangle_area)
            right_section += inside_rectangle(line1_first_third, rect_points[1], rect_points[2], line2_first_third,
                                              current_x, current_y, right_rectangle_area)

        left_percentage = left_section / object_vertex_length * 100
        print("\nleft %:", left_percentage)
        middle_percentage = middle_section / object_vertex_length * 100
        print("Middle %:", middle_percentage)
        right_percentage = right_section / object_vertex_length * 100
        print("Right %:", right_percentage)
        if left_percentage > 50:
            print("{} in Left".format(obj[1]))
            left_obj_list.append(obj)
        if middle_percentage > 50:
            print("{} in Middle".format(obj[1]))
            middle_obj_list.append(obj)
        if right_percentage > 50:
            print("{} in Right".format(obj[1]))
            right_obj_list.append(obj)


def sort_object_list(obj_list, startpoint_object):
    n = len(obj_list)
    swapped = False
    for i in range(n - 1):
        for j in range(0, n - i - 1):
            distance_j1 = startpoint_object.compute_point_cloud_distance(obj_list[j][0])
            distance_j2 = startpoint_object.compute_point_cloud_distance(obj_list[j + 1][0])

            min_distance_j1 = np.min(distance_j1)
            min_distance_j2 = np.min(distance_j2)

            if min_distance_j1 > min_distance_j2:
                swapped = True
                obj_list[j], obj_list[j + 1] = obj_list[j + 1], obj_list[j]

        if not swapped:
            return


def print_sorted_object_list(sorted_list):
    for obj in sorted_list:
        print(obj[1])


def return_coordinates(obstacle, distance_array):
    obstacle_np = np.asarray(obstacle.points)
    closest_index = np.argmin(distance_array, axis=0)
    closest_coordinates_from_object = obstacle_np[closest_index]
    return closest_coordinates_from_object


def get_horizontal_distance(self_object, target_object):
    first_array = np.asarray(self_object.points)
    second_array = np.asarray(target_object.points)

    # Initialize variables to store shortest distance and corresponding indices
    shortest_distance = np.inf
    first_array_index = -1
    second_array_index = -1

    # Loop over points in the first array
    for i, point1 in enumerate(first_array):
        # Calculate distances between point1 and all points in the second array
        point_distances = np.linalg.norm(point1 - second_array, axis=1)
        # Find the minimum distance and its index
        min_distance_index = np.argmin(point_distances)
        min_distance = point_distances[min_distance_index]
        # Update shortest distance and corresponding indices if necessary
        if min_distance < shortest_distance:
            shortest_distance = min_distance
            first_array_index = i
            second_array_index = min_distance_index

    return shortest_distance, first_array_index, second_array_index


def check_enough_space(side, wall, obj_list):
    print("\nTest {}:".format(side))
    print("{} object list length: {}".format(side, len(obj_list)))

    print("width:", width)
    node_list = []

    for i in range(len(obj_list)):
        print("\nobj{}: {}".format(i, obj_list[i][1]))
        node_list.append([])
        sub_list = node_list[i]
        sub_list.extend([obj_list[i]])
        # node_list.append(obj_list[i][0])
        # check wall can pass
        wall_min_distance, self_index, wall_index = get_horizontal_distance(obj_list[i][0], wall[0])
        print("obj{} to wall distance: {}".format(i, wall_min_distance))
        if wall_min_distance >= width:
            print("wall can pass")
            sub_list = node_list[i]
            sub_list.extend([self_index, wall, wall_index])

        # check middle can pass
        # find the closest middle object
        middle_obj_index = 0
        # initialize it from the first object in middle_obj_list
        min_middle_min_distance_result = get_horizontal_distance(obj_list[i][0],
                                                                 middle_obj_list[0][0])
        for x in range(len(middle_obj_list)):
            current_middle_obj_result = get_horizontal_distance(obj_list[i][0], middle_obj_list[x][0])
            # print("comparing to middle obj{} {}: {}".format(x, middle_obj_list[x][1], middle_obj_distance))
            if current_middle_obj_result[0] < min_middle_min_distance_result[0]:
                min_middle_min_distance_result = current_middle_obj_result
                middle_obj_index = x
        print("Shortest: obj{} {} to middle object{} {} distance: {}".format(i, obj_list[i][1], middle_obj_index,
                                                                             middle_obj_list[middle_obj_index][1],
                                                                             min_middle_min_distance_result[0]))
        min_middle_distance, self_index, target_index = min_middle_min_distance_result
        if min_middle_distance >= width:
            print("middle can pass")
            sub_list = node_list[i]
            sub_list.extend([self_index, middle_obj_list[middle_obj_index], target_index])

        # both wall and middle cannot pass, so this side cannot reach destination, the length is only 1.
        if len(node_list[i]) == 0:
            return node_list, False

    return node_list, True

    # # Add the destination point
    # stop_list.append(destination_center)
    # draw_line(stop_list)
    # return 1


def set_tree(node_list):
    """node_list[i] = [ [left object pcd, name], left object coord index, [target pcd, name], target coord index,
        left object coord index, [target pcd, name], target coord index,
        assume there are only 2 way to go (wall, middle) for each side

        Each node in tree should have [ [pcd, name], self_coord_index, "object-to-target"]
    """
    root = TreeNode([center, "start"])
    for i in range(len(node_list)):
        if len(node_list[i]) // 3 > 1:
            # put two node to the deepest leaf
            name = node_list[i][0][1] + "-" + node_list[i][2][1]
            node_info = [node_list[i][0], node_list[i][1], name]

            name2 = node_list[i][0][1] + "-" + node_list[i][5][1]
            node_info2 = [node_list[i][0], node_list[i][4], name2]
            root.add_children_to_deepest_leaves([node_info, node_info2])
        else:
            name = node_list[i][0][1] + node_list[i][2][1]
            node_info = [node_list[i][0], node_list[i][1], name]
            root.add_children_to_deepest_leaves([node_info])

    dest_node = [destination_center, 'destination']
    root.add_children_to_deepest_leaves([dest_node])
    paths = root.get_paths_to_leaves()
    print("Paths from root to each leaf:")
    for path in paths:
        print(path)
    return paths


def calculate_distance(x1, y1, x2, y2):
    """
    Calculates the distance between two points with (x, y) coordinates using the Euclidean distance formula.
    """
    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return distance


def find_shortest_path(path_list1, path1_can_pass, path_list2, path2_can_pass):
    if path1_can_pass and path2_can_pass:
        path_list1.extend(path_list2)
    if path1_can_pass and not path2_can_pass:
        path_list1 = path_list1
    if not path1_can_pass and path2_can_pass:
        path_list1 = path_list2
    if not path1_can_pass and not path2_can_pass:
        print("\n<<<<<<<<<<<<<<< NO PATH CAN REACH DESTINATION >>>>>>>>>>>>>>>")
        return

    total_distance = []
    paths_coord = []
    for i in range(len(path_list1)):
        total_distance.append(0)
        sub_list = []
        print("path{}: {}".format(i, path_list1[i]))
        for x in range(len(path_list1[i]) - 1):
            if x == 0:
                starting_array = path_list1[i][0][0]
                x1, y1 = starting_array[0], starting_array[1]
                print("starting point x1: {}, y1: {}".format(x1, y1))
            else:
                index = path_list1[i][x][1]
                first_pcd = path_list1[i][x][0][0]  # point cloud
                first_pcd_points = np.asarray(first_pcd.points)
                first_pcd_coord = first_pcd_points[index]
                x1, y1 = first_pcd_coord[0], first_pcd_coord[1]
                print("\nx1: {}, y1: {}".format(x1, y1))

            if x == len(path_list1[i]) - 2:
                destination_array = path_list1[i][len(path_list1[i]) - 1][0]
                x2, y2 = destination_array[0], destination_array[1]
                print("destination x2: {}, y2: {}".format(x2, y2))
            else:
                index = path_list1[i][x + 1][1]
                second_pcd = path_list1[i][x + 1][0][0]  # point cloud
                second_pcd_points = np.asarray(second_pcd.points)
                second_pcd_coord = second_pcd_points[index]
                x2, y2 = second_pcd_coord[0], second_pcd_coord[1]

            print("x2: {}, y2: {}".format(x2, y2))
            total_distance[i] += calculate_distance(x1, y1, x2, y2)
            sub_list.append(x1)
            sub_list.append(y1)
            if x == len(path_list1[i]) - 2:
                sub_list.append(x2)
                sub_list.append(y2)
        paths_coord.append(sub_list)
        print("path{} distance: {}".format(i, total_distance[i]))

    print("paths_coord:", paths_coord)

    shortest_distance = 99999
    shortest_distance_index = 0
    for i in range(len(total_distance)):
        distance = total_distance[i]
        if distance < shortest_distance:
            shortest_distance = distance
            shortest_distance_index = i
    print("path{} has the shortest distance: {}".format(shortest_distance_index, shortest_distance))
    print("\nshortest path coord:", paths_coord[shortest_distance_index])
    # draw line to plot the path
    shortest_coord_array = paths_coord[shortest_distance_index]
    for i in range(len(shortest_coord_array) // 2 - 1):
        x = i * 2
        coord_x1, coord_y1, coord_z1 = shortest_coord_array[x], shortest_coord_array[x + 1], 0.8
        coord_x2, coord_y2, coord_z2 = shortest_coord_array[x + 2], shortest_coord_array[x + 3], 0.8
        arr1 = [coord_x1, coord_y1, coord_z1]
        arr2 = [coord_x2, coord_y2, coord_z2]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(np.array([arr1, arr2]))
        line_set.lines = o3d.utility.Vector2iVector(np.array([[0, 1]]))
        display_list.append(line_set)


def append_display_list(obj_list):
    for obj in obj_list:
        display_list.append(obj[0])


"""
    All the core functions are called here
"""
get_section()
sort_object_list(left_obj_list, door)
sort_object_list(middle_obj_list, door)
sort_object_list(right_obj_list, door)
print("\nsorted left:")
print_sorted_object_list(left_obj_list)
print("\nsorted middle:")
print_sorted_object_list(middle_obj_list)
print("\nsorted right:")
print_sorted_object_list(right_obj_list)

print("")
left_node_list, left_can_pass = check_enough_space("left", westwall, left_obj_list)
print("left_node_list:", left_node_list)
print("left can pass?", left_can_pass)

print("")
right_node_list, right_can_pass = check_enough_space("right", eastwall, right_obj_list)
print("right_node_list:", right_node_list)
print("right can pass?", right_can_pass)

if left_can_pass:
    left_paths = set_tree(left_node_list)
else:
    left_paths = []
if right_can_pass:
    right_paths = set_tree(right_node_list)
else:
    right_paths = []
# Find the shortest path
find_shortest_path(left_paths, left_can_pass, right_paths, right_can_pass)

et = time.time()
time_diff = et - st
print("Time used:", time_diff, "seconds")
"""
    End: All the core functions are called here
"""


"""
    Change the corresponding variable name to visualize the 3D object in open3D,
    you may append more variable to the display_list
"""
display_list.append(door)
display_list.append(floor)

display_list.append(northwall)
display_list.append(eastwall[0])
display_list.append(southwall)
display_list.append(westwall[0])
"""
    End: Changing variable name
"""


"""
    The below commented lines is a simple 3d coordinate pointer, which points the direction of
    x, y, z, which the value increases in that way
"""
# The coordinate system
# points = np.array([[0.1, 0.1, 0.1], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
# colors = [[1, 1, 1], [1, 0, 0], [0, 1, 0], [0, 0, 1]]
# test_pcd = open3d.geometry.PointCloud()
# test_pcd.points = open3d.utility.Vector3dVector(points)
# test_pcd.colors = open3d.utility.Vector3dVector(colors)
# display_list.append(test_pcd)
"""
    End of coordinate pointer
"""

"""
    Append the obstacles in left, middle and right section to display list in open3D
"""
append_display_list(left_obj_list)
append_display_list(middle_obj_list)
append_display_list(right_obj_list)
"""
    End: Append the obstacles in left, middle and right section to display list in open3D
"""

o3d.visualization.draw_geometries(display_list)

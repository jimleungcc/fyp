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

    sorted_idx = np.argsort(floorbox_8points[:, 0])
    sorted_a = floorbox_8points[sorted_idx]
    # print("floor 8 points: ", sorted_a)
    # objectList.append(floor_box)


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


import numpy
import open3d as o3d

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

eastSide = []
eastSide.append(eastchair1)
eastSide.append(eastchair2)
eastSide.append(eastchair3)

# dist = westchair1.compute_point_cloud_distance(westwall)
# print("numpy min:", numpy.min(dist))
# print("numpy max:", numpy.max(dist))
# print("numpy avg:", numpy.mean(dist))


# Assume the object width is same as the door

# Get the AABB of the point cloud
aabb = door.get_axis_aligned_bounding_box()
# Get the width of the AABB
width = (aabb.get_max_bound()[0] - aabb.get_min_bound()[0]) / 2
print("Width of the door: ", width)

width = 0.2
possiblePath = 0


def checkEnoughSpace(side, wall, objList):
    canPass = True
    print("\nTest {}:".format(side))
    for i in range(len(objList)):
        distance = objList[i].compute_point_cloud_distance(wall)
        minDistance = numpy.min(distance)
        print("obj{} minDistance in {}: {}".format(i, side, minDistance))
        if minDistance < width:
            print("Obj{} not enough width in {}".format(i, side))
            canPass = False
            print("{} cannot pass".format(side))
            return 0
    if canPass:
        print("{} can pass".format(side))
        return 1


possiblePath += checkEnoughSpace("east", eastwall, eastSide)
possiblePath += checkEnoughSpace("west", westwall, westSide)

print("\npossible path:", possiblePath)

objectList = []

objectList.append(table)
objectList.append(door)
objectList.append(floor)

objectList.append(northwall)
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

o3d.visualization.draw_geometries(objectList)

from plyfile import PlyData, PlyElement
import open3d as o3d

# Load the PLY file
plydata = PlyData.read('westwall_combine.ply')

# Define the threshold value
y1 = -13.693006666666667
y2 = -15.447105333333335

y_coords = plydata['vertex'].data['y']
# left section
num_larger_y1 = sum(y > y1 for y in y_coords)
percentage_larger_y1 = num_larger_y1 / len(y_coords) * 100
print("Percentage of vertices with y-coordinate larger than", y1, ":", percentage_larger_y1, "%")


num_larger_y2 = sum (y > y2 for y in y_coords)
percentage_larger_y2 = num_larger_y2 / len(y_coords) * 100
print("larger than y2", percentage_larger_y2)

num_between = sum(y1 > y > y2 for y in y_coords)
percentage_between = num_between / len(y_coords) * 100
print("Percentage of vertices between", y1, "and", y2, ":", percentage_between, "%")

num_smaller_y2 = sum(y < y2 for y in y_coords)
percentage_smaller_y2 = num_smaller_y2 / len(y_coords) * 100
print("Percentage of vertices with y-coordinate smaller than", y2, ":", percentage_smaller_y2, "%")

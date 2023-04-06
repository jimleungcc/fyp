# import numpy as np
#
# # Create sample 2D arrays representing x and y coordinates
# first_array = np.array([[1, 2],
#                        [3, 4],
#                        [5, 6]])
#
# second_array = np.array([[7, 8],
#                         [9, 10],
#                         [11, 12],
#                         [13, 14]])
#
# # Initialize an empty list to store distances
# distances = []
#
# # Iterate over all points in first_array
# for point1 in first_array:
#     # Compute distances between point1 and all points in second_array
#     point_distances = np.linalg.norm(point1 - second_array, axis=1)
#     # Append the point_distances to the distances list
#     distances.append(point_distances)
#
# # Convert distances list to a NumPy array
# print(distances)
# distances = np.array(distances)
#
# # Get the shortest distance among all distances
# shortest_distance = np.min(distances)
#
# # Print the shortest distance
# print("Shortest Euclidean distance: ", shortest_distance)




import numpy as np

# Create sample 2D arrays representing x and y coordinates
first_array = np.array([[1, 2],
                       [3, 4],
                       [5, 6]])

second_array = np.array([[7, 8],
                        [9, 10],
                        [11, 12],
                        [13, 14]])

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

# Output the shortest distance and corresponding indices
print("Shortest Euclidean Distance: ", shortest_distance)
print("Index in First Array: ", first_array_index)
print("Index in Second Array: ", second_array_index)


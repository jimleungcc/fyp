import math

# Prompt the user to input the coordinates of the two endpoints of the line segment
x1, y1 = map(float, input("Enter the coordinates of the first point: ").split(','))
x2, y2 = map(float, input("Enter the coordinates of the second point: ").split(','))

# Define the endpoints of the line segment as tuples
p1 = (x1, y1)
p2 = (x2, y2)

# Calculate the length of the line segment
d = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

# Calculate the distance from the first point to the first 1/3 point
d_1_3 = (1/3) * d

# Calculate the slope of the line segment
m = (p2[1]-p1[1]) / (p2[0]-p1[0])

# Calculate the x-coordinate of the first 1/3 point
x_1_3_1 = p1[0] + d_1_3 / math.sqrt(1 + m**2)

# Calculate the y-coordinate of the first 1/3 point
y_1_3_1 = p1[1] + m * (d_1_3 / math.sqrt(1 + m**2))

# Calculate the distance from the first point to the second 1/3 point
d_2_3 = (2/3) * d

# Calculate the x-coordinate of the second 1/3 point
x_1_3_2 = p1[0] + d_2_3 / math.sqrt(1 + m**2)

# Calculate the y-coordinate of the second 1/3 point
y_1_3_2 = p1[1] + m * (d_2_3 / math.sqrt(1 + m**2))

# Print the coordinates of both 1/3 points
print(f"The coordinates of the first 1/3 point on the line segment are ({x_1_3_1}, {y_1_3_1})")
print(f"The coordinates of the second 1/3 point on the line segment are ({x_1_3_2}, {y_1_3_2})")

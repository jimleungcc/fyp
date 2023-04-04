import math


# Coordinates of the rectangle's vertices clockwise
def triangle_area(px, py, x1, y1, x2, y2):
    # Calculate the area of the triangle using the Shoelace Formula
    area = 0.5 * abs((x1 * (y2 - py) + x2 * (py - y1) + px * (y1 - y2)))
    return area


x1, y1 = -0.445642757544149, -17.466895820957014
x2, y2 = 0.03580410526995892, -16.519168562706312
x3, y3 = -5.974431629318904, -13.465993513891647
x4, y4 = -6.455878492133011, -14.41372077214235

# Length of the base
base_length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Height of the rectangle
height = math.sqrt((x3 - x2) ** 2 + (y3 - y2) ** 2)

# Area of the rectangle
area = base_length * height

# Print the area of the rectangle
print("The area of the rectangle is:", area, "square units.")

# Coordinates of the triangle's vertices
px, py = -1.877909, -13.668191

t1 = triangle_area(px, py, x1, y1, x2, y2)
t2 = triangle_area(px, py, x2, y2, x3, y3)
t3 = triangle_area(px, py, x3, y3, x4, y4)
t4 = triangle_area(px, py, x1, y1, x4, y4)

# Print the area of the triangle
print("t1", t1)
print("t2", t2)
print("t3", t3)
print("t4", t4)

area_sum_of_triangle = t1 + t2 + t3 + t4
print("total area", area_sum_of_triangle)

round_area = round(area, 5)
round_total_area = round(area_sum_of_triangle, 5)

print("rounded area", round_area)
print("rounded total area of triangles", round_total_area)

if round_area == round_total_area:
    print("Yes")
else:
    print("No")

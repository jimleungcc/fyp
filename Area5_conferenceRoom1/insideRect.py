import math

# Coordinates of the rectangle's vertices clockwise
# x1, y1 = 1, 3
# x2, y2 = 6, 8
# x3, y3 = 8, 6
# x4, y4 = 3, 1

x1, y1 = 0.998697831, -14.6237140
x2, y2 = -0.445642758, -17.4668958
x3, y3 = -6.45587849, -14.4137208
x4, y4 = -5.01153790, -11.5705390

# Length of the base
base_length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Height of the rectangle
height = math.sqrt((x3 - x2) ** 2 + (y3 - y2) ** 2)

# Area of the rectangle
area = base_length * height

# Print the area of the rectangle
print("The area of the rectangle is:", area, "square units.")


def triangle_area(px, py, x1, y1, x2, y2):
    # Calculate the area of the triangle using the Shoelace Formula
    area = 0.5 * abs((x1 * (y2 - py) + x2 * (py - y1) + px * (y1 - y2)))
    return area


# Coordinates of the triangle's vertices
px, py = 0, 0

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

if round_area == round_total_area:
    print("inside")
else:
    print("outside")
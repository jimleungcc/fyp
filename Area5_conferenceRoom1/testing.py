import math

# Function to calculate Euclidean distance
def calculate_euclidean_distance(x1, y1, x2, y2):
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance

# Loop to allow user to rerun the program
while True:
    # Get user input for coordinates of point 1
    x1 = float(input("Enter x-coordinate of point 1: "))
    y1 = float(input("Enter y-coordinate of point 1: "))

    # Get user input for coordinates of point 2
    x2 = float(input("Enter x-coordinate of point 2: "))
    y2 = float(input("Enter y-coordinate of point 2: "))

    # Calculate Euclidean distance
    distance = calculate_euclidean_distance(x1, y1, x2, y2)

    # Print the result
    print("The Euclidean distance between point 1 ({}, {}) and point 2 ({}, {}) is: {}".format(x1, y1, x2, y2, distance))

    # Ask user if they want to rerun the program
    rerun = input("Do you want to rerun the program? (y/n): ")
    if rerun.lower() != 'y':
        break

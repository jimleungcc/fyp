# Get the input file names from the user
input_file1 = input("Enter the name of the first PLY file: ")
input_file2 = input("Enter the name of the second PLY file: ")

# Get the output file name from the user
output_file = input("Enter the name of the output file: ")

# Open the first PLY file and read the header
with open(input_file1 + ".ply", 'r') as f:
    # Read the header lines
    header1 = []
    line = f.readline().strip()
    while line != 'end_header':
        header1.append(line)
        line = f.readline().strip()

    # Extract the number of vertices
    num_vertices1 = None
    for line in header1:
        if line.startswith('element vertex'):
            num_vertices1 = int(line.split()[2])
            break

    # Extract the vertex data
    vertices1 = []
    for i in range(num_vertices1):
        line = f.readline().strip()
        x, y, z, red, green, blue = map(float, line.split())
        vertices1.append((x, y, z, red, green, blue))

# Open the second PLY file and read the header
with open(input_file2 + ".ply", 'r') as f:
    # Read the header lines
    header2 = []
    line = f.readline().strip()
    while line != 'end_header':
        header2.append(line)
        line = f.readline().strip()

    # Extract the number of vertices
    num_vertices2 = None
    for line in header2:
        if line.startswith('element vertex'):
            num_vertices2 = int(line.split()[2])
            break

    # Extract the vertex data
    vertices2 = []
    for i in range(num_vertices2):
        line = f.readline().strip()
        x, y, z, red, green, blue = map(float, line.split())
        vertices2.append((x, y, z, red, green, blue))

# Combine the vertex data
combined_vertices = vertices1 + vertices2
num_combined_vertices = len(combined_vertices)

# Write the combined PLY file
with open(output_file + ".ply", 'w') as f:
    # Write the header lines
    f.write('ply\n')
    f.write('format ascii 1.0\n')
    f.write('comment Combined PLY file\n')
    f.write('element vertex ' + str(num_combined_vertices) + '\n')
    f.write('property float x\n')
    f.write('property float y\n')
    f.write('property float z\n')
    f.write('property uchar red\n')
    f.write('property uchar green\n')
    f.write('property uchar blue\n')
    f.write('end_header\n')

    # Write the vertex data
    for vertex in combined_vertices:
        f.write(str(vertex[0]) + ' ' + str(vertex[1]) + ' ' + str(vertex[2]) + ' ' +
                str(int(vertex[3])) + ' ' + str(int(vertex[4])) + ' ' + str(int(vertex[5])) + '\n')

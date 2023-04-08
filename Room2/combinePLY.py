# Get the input file names from the user
num_files = int(input("Enter the number of PLY files to combine: "))
input_files = []
for i in range(num_files):
    input_file = input("Enter the name of PLY file " + str(i + 1) + ": ")
    input_files.append(input_file)

# Get the output file name from the user
output_file = input("Enter the name of the output file: ")

# Combine the vertex data from all input files
combined_vertices = []
num_combined_vertices = 0
for input_file in input_files:
    # Open the PLY file and read the header
    with open(input_file + ".ply", 'r') as f:
        # Read the header lines
        header = []
        line = f.readline().strip()
        while line != 'end_header':
            header.append(line)
            line = f.readline().strip()

        # Extract the number of vertices
        num_vertices = None
        for line in header:
            if line.startswith('element vertex'):
                num_vertices = int(line.split()[2])
                break

        # Extract the vertex data
        vertices = []
        for i in range(num_vertices):
            line = f.readline().strip()
            x, y, z, red, green, blue = map(float, line.split())
            vertices.append((x, y, z, red, green, blue))

        # Combine the vertex data with the existing data
        combined_vertices += vertices
        num_combined_vertices += num_vertices

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

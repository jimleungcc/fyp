import os
import plyfile

# Get a list of all PLY files in the directory
ply_files = [f for f in os.listdir() if f.endswith('.ply')]

for ply_file in ply_files:
    # Open the binary PLY file in read mode
    with open(ply_file, 'rb') as f:
        # Read the PLY file header
        plydata = plyfile.PlyData.read(f)

    # Create the output file name
    output_file = 'zconverted_' + ply_file

    # Open the ascii PLY file in write mode
    with open(output_file, 'w') as f:
        # Write the PLY file header in ascii format
        element = plydata.elements[0]
        f.write('ply\n')
        f.write('format ascii 1.0\n')
        f.write('element {} {}\n'.format(element.name, len(element.data)))
        f.write('property {} {}\n'.format('double', 'x'))
        f.write('property {} {}\n'.format('double', 'y'))
        f.write('property {} {}\n'.format('double', 'z'))
        f.write('property {} {}\n'.format('uchar', 'red'))
        f.write('property {} {}\n'.format('uchar', 'green'))
        f.write('property {} {}\n'.format('uchar', 'blue'))
        f.write('element face 0\n')
        f.write('property list {} {} {}\n'.format('uchar', 'int', 'vertex_indices'))
        f.write('end_header\n')

        # Write the PLY file data in ascii format
        for data in element.data:
            f.write(' '.join(str(x) for x in data) + '\n')

    # Delete the original PLY file
    os.remove(ply_file)

    # Remove the prefix "zconverted_" for all PLYs
    os.rename(output_file, output_file.replace('zconverted_', ''))
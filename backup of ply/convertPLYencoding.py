import plyfile

# Open the binary PLY file in read mode
with open('door.ply', 'rb') as f:
    # Read the PLY file header
    plydata = plyfile.PlyData.read(f)

# Open the ascii PLY file in write mode
with open('output_file1.ply', 'w') as f:
    # Write the PLY file header in ascii format
    element = plydata.elements[0]
    f.write('ply\n')
    f.write('format ascii 1.0\n')
    f.write('element {} {}\n'.format(element.name, len(element.data)))
    # f.write('element {}\n'.format(element.name))
    f.write('property {} {}\n'.format('double', 'x'))
    f.write('property {} {}\n'.format('double', 'y'))
    f.write('property {} {}\n'.format('double', 'z'))
    f.write('property {} {}\n'.format('uchar', 'red'))
    f.write('property {} {}\n'.format('uchar', 'green'))
    f.write('property {} {}\n'.format('uchar', 'blue'))
    f.write('element face 0')
    f.write('property list {} {} {}\n'.format('uchar', 'int', 'vertex_indices'))
    f.write('end_header\n')

    # Write the PLY file data in ascii format
    for data in element.data:
        f.write(' '.join(str(x) for x in data) + '\n')

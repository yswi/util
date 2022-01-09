'''
compilation of python functions useful for loading data or files.
'''

 


def read_all_files_in_dir(mypath)
 
 '''
  read_all_files_in_dir : list all files in a directory in one line
  reference : https://stackoverflow.com/questions/3207219/how-do-i-list-all-files-of-a-directory
  input : path
  output : list of file names
 '''
 
  from os import walk
  _, _, filenames = next(walk(mypath))
  return filenames
 
def read_xyz(filename):
'''
   read_xyz : Read *.xyz file (point cloud) 
'''
 
    atoms = []
    coordinates = []

    xyz = open(filename)
    n_atoms = int(xyz.readline())
    title = xyz.readline()
    for line in xyz:
        atom,x,y,z = line.split()
        atoms.append(atom)
        coordinates.append([float(x), float(y), float(z)])
    xyz.close()

    if n_atoms != len(coordinates):
        raise ValueError("File says %d atoms but read %d points." % (n_atoms, len(coordinates)))

   return atoms, coordinates

                         
def write_list_as_txt(lst, path = None):
    if path is None:
        path = '.' # Save in current directory
                         
    with open(path, "w") as f:
        for ele in lst:
            f.write(ele)
            f.write(',\n')

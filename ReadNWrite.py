'''
compilation of python functions useful for loading data or files.
'''

 
 
 '''
  purpose : list all files in a directory in one line
  reference : https://stackoverflow.com/questions/3207219/how-do-i-list-all-files-of-a-directory
  input : path
  output : list of file names
 '''

def all_files_in_dir(mypath)

  from os import walk
  _, _, filenames = next(walk(mypath))
  return filenames

def read_xyz(filename):
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
        raise ValueError("File says %d atoms but read %d points." % (n_atoms, len(coordinates))

   return atoms, coordinates

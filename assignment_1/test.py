import os
path_name = "input.txt"
if os.path.isfile(path_name):
  f = open(path_name)
  #Execute other file operations here
  print(f.readlines())
  f.close()
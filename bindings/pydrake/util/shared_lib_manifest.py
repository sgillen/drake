import os
import subprocess

import pydrake

def parse_out(root, out):
    root_dir = os.path.dirname(root.__file__)
    lines = out.split("\n")
    for line in lines:
        delim = " => "
        if " => " not in line:
            continue
        base, path = line.strip().split(delim)
        print(base)
        if path.startswith(root_dir):
            print(path)
            print(os.path.relpath(path, root_dir))
        print("---")

print(os.getcwd())

out = subprocess.check_output(
    "ldd {}".format(pydrake.common.__file__), shell=True)
print(parse_out(pydrake, out))

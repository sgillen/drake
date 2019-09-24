#!/usr/bin/python3
import sys
import os

os.chdir(os.path.dirname(__file__))

reps = [
    ("../bazel-bin", "${bazel_bin}"),
    ("../bazel-out", "${bazel_out}"),
    ("..", "${PWD}"),
]

for line in sys.stdin.readlines():
    line = line.rstrip()
    prefix = "openat("
    if not line.startswith(prefix):
        continue
    if ".so" not in line:
        continue
    if not line.endswith(" = 3"):
        continue
    for real, fake in reps:
        real = os.path.realpath(real)
        line = line.replace(real, fake)
    print(line)

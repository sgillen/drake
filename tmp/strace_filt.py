#!/usr/bin/python3
import argparse
import os
import sys

assert __name__ == "__main__"
parser = argparse.ArgumentParser()
parser.add_argument("input", type=argparse.FileType("r"))
parser.add_argument("output", type=argparse.FileType("w"))
args = parser.parse_args()

os.chdir(os.path.dirname(__file__))

reps = [
    ("../bazel-bin", "${bazel_bin}"),
    ("../bazel-out", "${bazel_out}"),
    ("..", "${PWD}"),
]

out = []
with args.input:
    for line in args.input.readlines():
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
        out.append(line)

with args.output:
    for line in out:
        print(line, file=args.output)

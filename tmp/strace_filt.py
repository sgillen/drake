#!/usr/bin/python3
import argparse
import os
import sys

assert __name__ == "__main__"
parser = argparse.ArgumentParser()
parser.add_argument("input", type=argparse.FileType("r"))
parser.add_argument("output_raw", type=argparse.FileType("w"))
parser.add_argument("output_sorted", type=argparse.FileType("w"))
args = parser.parse_args()

os.chdir(os.path.dirname(__file__))

reps = [
    ("../bazel-bin", "${bazel_bin}"),
    ("../bazel-out", "${bazel_out}"),
    ("./venv", "${venv}"),
    ("..", "${workspace}"),
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
            real = os.path.abspath(real)
            line = line.replace(real, fake)
        out.append(line)

for f, lines in ((args.output_raw, out), (args.output_sorted, sorted(out))):
    with f:
        for line in lines:
            print(line, file=f)

#!/usr/bin/python3
import argparse
import os
import re
import sys

assert __name__ == "__main__"
parser = argparse.ArgumentParser()
parser.add_argument("input", type=argparse.FileType("r"))
parser.add_argument("output_raw", type=argparse.FileType("w"))
parser.add_argument("output_sorted", type=argparse.FileType("w"))
args = parser.parse_args()

os.chdir(os.path.dirname(__file__))

def path_rep(p, fake):
    p_abs = os.path.abspath(p)
    p_real = os.path.realpath(p)
    return lambda x: x.replace(p_abs, fake).replace(p_real, fake)

def re_rep(pattern, fake):
    pattern = re.compile(pattern)
    return lambda x: re.sub(pattern, fake, x)

reps = [
    path_rep("../bazel-bin", "${bazel_bin}"),
    path_rep("../bazel-out", "${bazel_out}"),
    path_rep("./venv", "${venv}"),
    path_rep("..", "${workspace}"),
    re_rep(r"\b0x[0-9a-fA-F]+\b", "<address>"),
    re_rep(r"\b[0-9]{4,}\b", "<num>"),
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
        for rep in reps:
            line = rep(line)
        out.append(line)

for f, lines in ((args.output_raw, out), (args.output_sorted, sorted(out))):
    with f:
        for line in lines:
            print(line, file=f)

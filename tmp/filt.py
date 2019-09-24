#!/usr/bin/python3
import argparse
import os
import re
import sys

assert __name__ == "__main__"
parser = argparse.ArgumentParser()
parser.add_argument("input", type=argparse.FileType("r"))
parser.add_argument("output_name", type=str)
parser.add_argument("--for_strace", action="store_true")
args = parser.parse_args()

os.chdir(os.path.dirname(__file__))

def path_rep(p, fake):
    p_abs = os.path.abspath(p)
    p_real = os.path.realpath(p)
    return lambda x: x.replace(p_abs, fake).replace(p_real, fake)

def re_rep(pattern, fake):
    pattern = re.compile(pattern)
    return lambda x: re.sub(pattern, fake, x)

def strace_filt(line):
    prefix = "openat("
    if not line.startswith(prefix):
        return
    if ".so" not in line:
        return
    if not line.endswith(" = 3"):
        return
    return line

reps = [
    str.rstrip,
    path_rep("../bazel-bin", "${bazel_bin}"),
    path_rep("../bazel-out", "${bazel_out}"),
    path_rep("./venv", "${venv}"),
    path_rep("..", "${workspace}"),
    re_rep(r"\b0x[0-9a-fA-F]+\b", "<address>"),
    re_rep(r"\b[0-9]{4,}\b", "<num>"),
]

if args.for_strace:
    reps.insert(1, strace_filt)

out = []
with args.input:
    for line in args.input.readlines():
        for rep in reps:
            line = rep(line)
            if line is None:
                break
        else:
            out.append(line)

if args.for_strace:
    for mode, lines in (("raw", out), ("sorted", sorted(out))):
        with open(f"{args.output_name}.{mode}.txt", "w") as f:
            for line in lines:
                print(line, file=f)
else:
    with open(f"{args.output_name}.txt", "w") as f:
        for line in out:
            print(line, file=f)   

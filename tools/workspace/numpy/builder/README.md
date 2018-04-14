# Building Experimental NumPy

To generate a `*.whl` file in `./build/`, that can then be redistributed
/ used in Bazel.

## Ubuntu

To build an experimental NumPy for Ubuntu 16.04, run:

    ./build_ubuntu.sh

## Mac

Ensure that you have the requirements equivalent to those in `Dockerfile`, then
run:

    ./build_mac.sh

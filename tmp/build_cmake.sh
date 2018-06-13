#!/bin/bash
set -eux

cd $(dirname $0)
bazel build --compiler=gcc-5 -c dbg @vtk//:vtk

prefix_dir=$(cd ../bazel-drake/external/vtk && pwd)
test -d ${prefix_dir}

rm -rf build && mkdir build && cd build

# Symlink for datafiles.
ln -s .. tmp

cmake .. \
    -DCMAKE_PREFIX_PATH=${prefix_dir} \
    -DCMAKE_BUILD_TYPE=Debug
make -j

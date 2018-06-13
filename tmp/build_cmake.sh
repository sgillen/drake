#!/bin/bash
set -eux

cd $(dirname $0)
prefix_dir=${PWD}/../bazel-drake/external/vtk
test -d ${prefix_dir}

rm -rf build && mkdir build && cd build

# Symlink for datafiles.
ln -s .. tmp

cmake -DCMAKE_PREFIX_PATH=${prefix_dir} -DCMAKE_BUILD_TYPE=Debug ..
make -j
./OSPRayPathTracer

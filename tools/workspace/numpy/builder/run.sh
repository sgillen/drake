#!/bin/bash

# Builds docker container which builds 

cd $(dirname $0)

docker build -t numpy_builder .

mkdir -p build
docker run --rm \
    -v ${PWD}/build:/output \
    numpy_builder \
    /build.sh /output https://github.com/numpy/numpy master

# Example installation via `pip`:
#   pip install --prefix ${PWD}/tmp numpy*.whl --ignore-installed
# Note that the `whl` is a zip archive, where the `numpy` directory can be
# used.

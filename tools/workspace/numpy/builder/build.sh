#!/bin/bash
set -e -u -x

# Creates and runs a Docker container which builds NumPy for Ubuntu 16.04.

cd $(dirname $0)

docker build -t numpy_builder .

mkdir -p build
docker run --rm \
    -v ${PWD}/build:/output \
    numpy_builder \
    /build_in_docker.sh /output https://github.com/numpy/numpy pull/10898/head

cat <<EOF
NumPy wheel file built.

To install to a prefix for direct testing:
    pip install --prefix \${PWD}/tmp numpy*.whl --ignore-installed
EOF

#!/bin/bash
set -eux

cd $(dirname $0)

rm -rf build

docker build -t drake-vtk .

mkdir build
docker run -v ${PWD}/build:/build drake-vtk \
    bash -c 'cp /opt/drake/*.tar.gz /build'

#!/bin/bash
set -eux

cd $(dirname $0)

rm -rf build

docker build -t drake-vtk .

mkdir build
docker run drake-vtk -v ~+/build:/opt/drake

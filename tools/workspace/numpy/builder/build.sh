#!/bin/bash
set -e -x -u

output_dir=${1}
repo=${2}
commit=${3}

git clone ${repo} /numpy

cd /numpy
git checkout -f ${commit}
python setup.py bdist_wheel
cp ./dist/numpy*.whl ${output_dir}

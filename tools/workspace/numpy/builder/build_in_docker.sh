#!/bin/bash
set -e -x -u

# Builds NumPy in docker
output_dir=${1}
repo=${2}
commit=${3}
shift; shift; shift

git clone ${repo} /numpy
cd /numpy

# Checkout specified commit, accommodating a PR if specified.
if [[ ${commit} =~ ^pull/.*$ ]]; then
    git fetch origin ${commit}
    git checkout FETCH_HEAD
else
    git checkout -f ${commit}
fi

python setup.py bdist_wheel "$@"
wheel_file=./dist/numpy*.whl

# Briefly print out NumPy version:
python -m virtualenv env
source env/bin/activate
python -m pip install ${wheel_file}
python -c 'import numpy; print(numpy.version.full_version)'

# Copy to directory, which should be a mounted volume.
cp ${wheel_file} ${output_dir}

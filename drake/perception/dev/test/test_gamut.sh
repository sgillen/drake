#!/bin/bash
set -e -x -u

cd $(dirname $0)/..
target=:point_cloud_test

t() {
    config=${1}
    bazel run --config ${config} ${target}
    bazel run -c dbg --config ${config} ${target}
}

# Nominal
t everything
# Checks
t memcheck
t asan
t msan
t lsan
t tsan
t ubsan

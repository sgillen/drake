#!/bin/bash
set -e -x -u

cd $(dirname $0)

t() {
    config=${1}
    bazel run --config ${config} :ref_map_tmp
    bazel run -c dbg --config ${config} :ref_map_tmp
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

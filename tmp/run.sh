#!/bin/bash
set -ux

file=$1

cd $(dirname $0)/..

bazel run --config=memcheck -c dbg //examples/kuka_iiwa_arm:kuka_simulation_test
echo $?

ldd bazel-bin/examples/kuka_iiwa_arm/kuka_simulation_test | sort > ${file}

#!/bin/bash
set -eux

cd $(dirname $0)

prefix=$1
outdir=${PWD}/${prefix}
mkdir -p ${outdir}

ldd-filt() {
    sort | regex_sub.py '\(0x[\da-f]+\)' '' -
}

cd ..
set +e
bazel build //examples/kuka_iiwa_arm:kuka_simulation_test
echo $?
set -e

objdump --all-headers -T -R bazel-bin/examples/kuka_iiwa_arm/kuka_simulation_test | c++filt  > ${outdir}/objdump_binary.txt
objdump --all-headers -T -R bazel-bin/tools/install/libdrake/libdrake.so | c++filt > ${outdir}/objdump_libdrake.txt

ldd bazel-bin/examples/kuka_iiwa_arm/kuka_simulation_test | ldd-filt > ${outdir}/ldd_binary.txt

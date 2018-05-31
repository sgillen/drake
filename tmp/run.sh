#!/bin/bash
set -ux

prefix=$1

cd $(dirname $0)/..

bazel run --config=memcheck -c dbg //examples/kuka_iiwa_arm:kuka_simulation_test
echo $?

outdir=tmp/${prefix}
mkdir -p ${outdir}

filt() {
    sort | regex_sub.py '\(0x[\da-f]+\)' '' -
}

ldd bazel-bin/examples/kuka_iiwa_arm/kuka_simulation_test | filt > ${outdir}/bin.txt
ldd bazel-bin/examples/kuka_iiwa_arm/../../_solib_k8/_U_S_Stools_Sinstall_Slibdrake_Cdrake_Ushared_Ulibrary___Utools_Sinstall_Slibdrake/libdrake.so | filt > ${outdir}/libdrake_install.txt
ldd bazel-bin/examples/kuka_iiwa_arm/../../_solib_k8/_U_S_Sexamples_Skuka_Uiiwa_Uarm_Ckuka_Usimulation_Utest___Utools_Sinstall_Slibdrake/libdrake.so | filt > ${outdir}/libdrake_example.txt

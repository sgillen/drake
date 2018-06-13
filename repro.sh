#!/bin/bash
set -eux

use_cmake=1

cd $(dirname $0)

if [[ ${use_cmake} -eq 1 ]]; then
    ./tmp/build_cmake.sh
    bin=tmp/build/rgbd_ospray_minimal
else
    bazel build --compiler=gcc-5 -c dbg //tmp:rgbd_ospray_minimal
    bin=bazel-bin/tmp/rgbd_ospray_minimal
fi

if [[ $(sysctl kernel.core_pattern) != "kernel.core_pattern = /tmp/core_dump" ]]; then
    echo 'Please run `sysctl -w kernel.core_pattern=/tmp/core_dump` for debugging.'
    exit 1;
fi
ulimit -c unlimited

rm -f /tmp/core_dump

set +e
(
    set -e
    # Normally takes about ~30 executions to produce segfault. Can range
    # anywhere from 2-50 (or more).
    for i in $(seq 300); do
        echo "[ $i ]"
        ${bin}
    done
)
set -e

# Postmortem
gdb -ex bt -ex q ${bin} /tmp/core_dump

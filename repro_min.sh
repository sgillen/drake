#!/bin/bash
set -eux

bazel build --compiler=gcc-5 -c dbg //tmp:rgbd_ospray_minimal

bin=bazel-bin/tmp/rgbd_ospray_minimal

if [[ $(sysctl kernel.core_pattern) != "kernel.core_pattern = /tmp/core_dump" ]]; then
    echo 'Please run `sysctl -w kernel.core_pattern=/tmp/core_dump` for debugging.'
    exit 1;
fi
ulimit -c unlimited

rm /tmp/core_dump

set +e
(
    set -e
    for i in $(seq 100); do
        echo "[ $i ]"
        ${bin}
    done
)
set -e

# Postmortem
gdb -ex bt -ex q ${bin} /tmp/core_dump

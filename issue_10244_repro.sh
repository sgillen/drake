#!/bin/bash
set -eux -o pipefail

bazel build -c dbg \
    //bindings/pydrake/systems:issue_10255_min_repro_static \
    //bindings/pydrake/systems:issue_10255_min_repro_shared

set +e
./bazel-bin/bindings/pydrake/systems/issue_10255_min_repro_static | tee output_static.txt
./bazel-bin/bindings/pydrake/systems/issue_10255_min_repro_shared | tee output_shared.txt
set -e

_bazel_out=$(readlink -f ./bazel-out)
normalize() {
    sed -r -e "s#${PWD}#\${PWD}#g" -e 's#0x[0-9a-g]+#[address]#g' -e "s#${_bazel_out}#\${PWD}/bazel-out#g"
}

ldd bazel-bin/bindings/pydrake/systems/issue_10255_min_repro_static | sort | normalize > ldd_static.txt
ldd bazel-bin/bindings/pydrake/systems/issue_10255_min_repro_shared | sort | normalize > ldd_shared.txt

set +e
env -i strace -s 256 ./bazel-bin/bindings/pydrake/systems/issue_10255_min_repro_static 2>&1 | normalize | tee strace_static.txt
env -i strace -s 256 ./bazel-bin/bindings/pydrake/systems/issue_10255_min_repro_shared 2>&1 | normalize | tee strace_shared.txt
set -e

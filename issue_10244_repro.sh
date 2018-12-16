#!/bin/bash
set -eux -o pipefail

bazel build \
    //bindings/pydrake/systems:issue_10255_min_repro_static \
    //bindings/pydrake/systems:issue_10255_min_repro_shared

set +e
./bazel-bin/bindings/pydrake/systems/issue_10255_min_repro_static | tee output_static.txt
./bazel-bin/bindings/pydrake/systems/issue_10255_min_repro_shared | tee output_shared.txt
set -e

normalize() {
    sort | sed -r -e "s#${PWD}#\$PWD#g" -e 's#\(0x[0-9a-g]+\)#([address])#g'
}

ldd bazel-bin/bindings/pydrake/systems/issue_10255_min_repro_static | normalize > ldd_static.txt
ldd bazel-bin/bindings/pydrake/systems/issue_10255_min_repro_shared | normalize > ldd_shared.txt

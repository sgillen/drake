#!/bin/bash
set -eux -o pipefail

bazel build \
    //bindings/pydrake/systems:issue_10255_min_repro_static \
    //bindings/pydrake/systems:issue_10255_min_repro_shared

./bazel-bin/bindings/pydrake/systems/issue_10255_min_repro_static
./bazel-bin/bindings/pydrake/systems/issue_10255_min_repro_shared

ldd bazel-bin/bindings/pydrake/systems/issue_10255_min_repro_static | sort | sed "s#${PWD}#\$PWD#g" > ldd_static.txt
ldd bazel-bin/bindings/pydrake/systems/issue_10255_min_repro_shared | sort | sed "s#${PWD}#\$PWD#g" > ldd_shared.txt

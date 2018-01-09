#!/bin/bash
set -e -u -x

workspace_file=${1}
cd $(dirname ${workspace_file})

# This message will only show up if symlinks are present and the user runs this
# test directly. If run via `test ...`, then they will get an obscure Bazel
# error "WARNING: Failed to get information about path..." because it will try
# and parse the symlinks in the local repository.
if [[ -L bazel-bin ]]; then
    cat >&2 <<EOF
Bazel struggles with its own symlinks with nested workspaces.
Please remove them by going to 'tools/external_data/workspace', and execute
   ./remove_bazel_symlinks.sh
This removes the polluting symlinks under the script's directory.
EOF
    exit 1
fi

# Run all the tests.
# Since this is pure Python + Bazel, we shouldn't care about propagating any
# configuration flags.
bazel test //...

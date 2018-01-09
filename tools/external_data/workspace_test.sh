#!/bin/bash
set -e -u -x

# echo "$@"
pwd
# exit 1

workspace_dir=${1}
if [[ -f ${workspace_dir} ]]; then
    workspace_dir=$(dirname ${workspace_dir})
fi
cd ${workspace_dir}

# Make sure we're where we want.
grep 'bazel_external_data_pkg' WORKSPACE

# Unfortunately, it's hard to get Bazel to produce a symlimk forest in
# `workspace/...`. It has trouble `glob`ing "workspace/" because of package
# restrictions.
# On top of that, even if it could, you cannot ignore `bazel-*` symlinks
# in local nested workspaces via `glob`.

if [[ -L bazel-bin ]]; then
    cat >&2 <<EOF
Bazel struggles with its own symlinks. Please remove them.
To do so, change to tools/external_data/workspace, and execute
   ./clear.sh
This does not clean the build, but simply removes the polluting symlinks under
this directory.
EOF
    exit 1
fi

# Run all the tests.
# Since this is pure Python + Bazel, we shouldn't care about propagating any
# configuration flags.
bazel test //...

# Ensure that we wipe out symlinks afterwards so that it doesn't fudge any
# upstream stuff.
# This will fail in `bazel test` as a read-only filesystem, but will succeed
# in `bazel run`.
./clear.sh || :

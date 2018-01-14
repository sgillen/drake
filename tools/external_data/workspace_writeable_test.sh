#!/bin/bash
set -e -u

# Copy necessary srcs to create a (set of) workspace(s) from an existing Bazel
# workspace. For meta-testing Bazel workflows.

# Prevent from running outside of Bazel.
if [[ ! $(basename $(dirname ${PWD})) =~ .*\.runfiles ]]; then
    echo "Must be run from within Bazel"
    exit 1
fi

readlink-py() {
    python -c 'import os, sys; print(os.path.realpath(sys.argv[1]))' ${1};
}

cmd=${1}
pkg_anchor=${2}
pkg_reldir=$(dirname ${pkg_anchor})

tmp_base=/tmp/bazel_workspace_test
mkdir -p ${tmp_base}
# TODO: Have this use `TEST_TMPDIR`
export WORKSPACE_TMP=$(mktemp -d -p ${tmp_base})

# Copy what's needed for a modifiable `bazel_pkg_advanced_test` directory.
mock_dir=${WORKSPACE_TMP}/mock_workspace

mkdir -p ${mock_dir}
# Do not use `$(locations ...)` is it won't easily give transitive files.
# Rely on `.runfiles` tree giving everything that we want.
srcs=$(find . -type f -o -type l)
for src in ${srcs}; do
    subdir=$(dirname ${src})
    mkdir -p ${mock_dir}/${subdir}
    # TODO: Consider designating `read-only` portions, use symlinks?
    cp $(readlink-py ${src}) ${mock_dir}/${subdir}
done

# Create mock drake/WORKSPACE file.
cd ${mock_dir}
! test -f ./WORKSPACE
echo 'workspace(name = "drake")' > ./WORKSPACE

# Change to the workspace directory.
cd ${mock_dir}/${pkg_reldir}
# Ensure path to Drake is corrected.
sed -i "s#path = .*,#path = \"${mock_dir}\",#g" ./WORKSPACE
# Get rid of Bazel symlinks if they exist.
rm bazel-* || :

# Execute command.
eval ${cmd}

#!/bin/bash
set -e -u -x

cd "${1}"
# Make sure we're where we want.
grep 'bazel_external_data_pkg' WORKSPACE
# Run all the tests.
bazel test //...
# Ensure that we wipe out symlinks so that it doesn't fudge any upstream stuff.
./clear.sh

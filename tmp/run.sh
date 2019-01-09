#!/bin/bash
set -eux -o pipefail

cd $(dirname $0)/..

# find . -name '*.h' -o -name '*.cc' | xargs -n 2000 python tmp/reformat_all.py -i

diffed() {
    git diff --name-only $(git merge-base upstream/master HEAD) HEAD
}

master=~/proj/tri/repo/branches/drake/master/drake
diffed | xargs ${master}/bazel-bin/tools/lint/clang-format-includes
${master}/bazel-bin/tools/lint/buildifier multibody/tree/BUILD.bazel

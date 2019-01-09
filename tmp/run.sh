#!/bin/bash
set -eux -o pipefail

cd $(dirname $0)/..

# find . -name '*.h' -o -name '*.cc' | xargs -n 2000 python tmp/reformat_all.py -i

master=~/proj/tri/repo/branches/drake/master/drake
find . -name '*.h' -o -name '*.cc'  | xargs -P 20 -n 100 ${master}/bazel-bin/tools/lint/clang-format-includes

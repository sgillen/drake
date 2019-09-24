#!/bin/bash
set -eux -o pipefail

cd $(dirname $0)

if [[ ! -d ./venv ]]; then
    python3 -m virtualenv -p python3 ./venv --system-site-packages
    ./venv/bin/pip install torch==1.0.0
fi

python=${PWD}/venv/bin/python

bazel build --python_path=${python} --action_env=DRAKE_PYTHON_BIN_PATH=${python} \
    //tmp:repro_issue12073
bin=../bazel-bin/tmp/repro_issue12073

${bin}
${bin} --torch_first

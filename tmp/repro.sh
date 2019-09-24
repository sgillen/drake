#!/bin/bash
set -eux -o pipefail

bash-isolate() {
     "$@";
}
# Ensure we have no external environment.
if [[ -z ${_isolate:+D} ]]; then
    exec env -i \
        _isolate=1 \
        HOME=$HOME DISPLAY=$DISPLAY SHELL=$SHELL TERM=$TERM USER=$USER PATH=/usr/local/bin:/usr/bin:/bin \
        bash --norc $0 "$@"
fi

cd $(dirname $0)

if [[ ! -d ./venv ]]; then
    python3 -m virtualenv -p python3 ./venv --system-site-packages
    ./venv/bin/pip install torch==1.0.0
fi

python=${PWD}/venv/bin/python

cat > ../user.bazelrc <<EOF
build --python_path=${python}
build --action_env=DRAKE_PYTHON_BIN_PATH=${python}
EOF


bazel build --announce_rc //tmp:repro_issue12073
bin=../bazel-bin/tmp/repro_issue12073

strace -o /tmp/strace.txt ${bin}; ./strace_filt.py /tmp/strace.txt ./strace_last.{raw,sorted}.txt
strace -o /tmp/strace.txt ${bin} --torch_first || :; ./strace_filt.py /tmp/strace.txt ./strace_first.{raw,sorted}.txt

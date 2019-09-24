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

# Hack `torch.__init__` to only import `torch._C`.
cat > ./venv/lib/python3.6/site-packages/torch/__init__.py <<'EOF'
# HACKED
print("Using hacked torch.__init__")
# Copied + simplified from original
import os
import sys
import numpy as _np
sys.setdlopenflags(os.RTLD_GLOBAL | os.RTLD_LAZY)
# Skipping: `import torch._nvrtc`
import torch._C
EOF

cat > ../hack.bazelrc <<EOF
build --python_path=${python}
build --action_env=DRAKE_PYTHON_BIN_PATH=${python}
build --cxxopt=-D_GLIBCXX_USE_CXX11_ABI=0
EOF


bazel build --announce_rc //tmp:repro_issue12073
bin=../bazel-bin/tmp/repro_issue12073
export PYTHONUNBUFFERED=1

run() {
    label=$1
    shift
    { strace -o /tmp/strace.txt "$@" || :; } 2>&1 | tee /tmp/output.txt
    ./filt.py /tmp/output.txt ./output_${label}
    ./filt.py --for_strace /tmp/strace.txt ./strace_${label}
}

run last ${bin}  # This should work
run first ${bin} --torch_first

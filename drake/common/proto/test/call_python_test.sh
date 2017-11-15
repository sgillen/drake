#!/bin/bash
set -e -u

cc_bin=${1}
py_client=${2}

if [[ ! -e /tmp/matlab_rpc ]]; then
    mkfifo /tmp/matlab_rpc
fi

# Start Python binary in the background.
${py_client} &

# Execute C++.
${cc_bin}

# When this is done, Python client should exit.
wait
echo "[ Done ]"

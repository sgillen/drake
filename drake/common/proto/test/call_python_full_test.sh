#!/bin/bash
set -e -u

cc_bin=${1}
py_client_cli=${2}

if [[ ! -e /tmp/python_rpc ]]; then
    mkfifo /tmp/python_rpc
fi

# Start Python binary in the background.
${py_client_cli} --no_loop &
pid=$!
# Execute C++.
${cc_bin}
# When this is done, Python client should exit.
wait $! || { echo "ERROR: Python client did not exit successfully."; exit 1; }

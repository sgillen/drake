#!/bin/bash
set -e -u

cc_bin=${1}
py_client_cli=${2}
py_client_example=${3}

if [[ ! -e /tmp/matlab_rpc ]]; then
    mkfifo /tmp/matlab_rpc
fi


# Start Python binary in the background.
${py_client_cli} --no_loop --no_threading &
# Execute C++.
${cc_bin}
# When this is done, Python client should exit.
wait

# # Do this again, but with example script to partially consume stuff.
# ${py_client_example} &
# ${cc_bin}
# wait

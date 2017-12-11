#!/bin/bash
set -e -u

# @file
# @brief Tests the `call_python_client` CLI and `call_python` test together.

cc_bin_flags=
while [[ $# -gt 2 ]]; do
    case ${1} in
        --no_plotting)
            cc_bin_flags='--gtest_filter=-TestCallPython.Plot*'
            shift;;
        *)
            echo "Bad argument: ${1}" >&2
            exit 1;;
    esac
done

cc_bin=${1}
py_client_cli=${2}

filename=/tmp/python_rpc
# if [[ ! -e ${filename} ]]; then
#     mkfifo ${filename}
# fi

py-error() {
    echo "ERROR: Python client did not exit successfully."
    exit 1
}

should-fail() {
    echo "This should have failed!"
    exit 2
}

sub-tests() {
    # Execute sub-cases.
    func=${1}
    # Sub-case 1: Nominal
    # @note This setup assumes other things succeeded.
    echo -e "\n[ ${func}: nominal ]"
    do-setup 0 0
    ${func}
    # # Sub-case 2: With Error
    # echo -e "\n[ ${func}: with_error ]"
    # do-setup 1 0
    # ${func}
    # # Sub-case 3: With Error + Stop on Error
    # echo -e "\n[ ${func}: with_error + stop_on_error ]"
    # do-setup 1 1
    # ${func}
}

py-check() {
    # Check result of Python process.
    if [[ ${py_fail} -eq 0 ]]; then
        # Should succeed.
        wait ${pid} || py-error
    else
        # Should fail.
        # TODO(eric.cousineau): File / find bug in Bash for this; this behaves
        # differently depending on how this is placed in a function.
        { wait ${pid} && should-fail; } || :
    fi
}

do-setup() {
    # usage: do-setup PY_FAIL PY_STOP_ON_ERROR
    py_fail=${1}
    py_stop_on_error=${2}

    cc_flags=
    if [[ ${py_fail} -eq 1 ]]; then
        cc_flags='--with_error'
    fi
    py_flags=
    if [[ ${py_stop_on_error} -eq 1 ]]; then
        py_flags='--stop_on_error'
    fi

    rm -f ${filename}
    touch ${filename}
}

# Execute tests.

no_threading-no_loop() {
    # Execute C++.
    ${cc_bin} ${cc_bin_flags} ${cc_flags}
    # Start Python binary in the background.
    ${py_client_cli} --no_threading --no_loop ${py_flags} &
    pid=$!
    # When this is done, Python client should exit.
    py-check
}
# sub-tests no_threading-no_loop

threading-no_loop() {
    ${cc_bin} ${cc_bin_flags} ${cc_flags}
    ${py_client_cli} --no_loop ${py_flags} &
    pid=$!
    py-check
}
# sub-tests threading-no_loop

run_under=""  #"operf --session-dir=/tmp/oprofile"

threading-loop() {
    # Execute client twice.
    echo "[cc start]"
    ${cc_bin} ${cc_bin_flags} ${cc_flags}
    ${cc_bin} ${cc_bin_flags} ${cc_flags}
    # Use `exec` so that we inherit SIGINT handlers.
    # @ref https://stackoverflow.com/a/44538799/7829525
    exec ${run_under} ${py_client_cli} ${py_flags} &
    pid=$!
    echo "[cc done]"
    if [[ ${py_stop_on_error} -ne 1 ]]; then
        # Wait for a small bit, then kill the client with Ctrl+C.
        # This is necessary to permit the client to finish processing.
        sleep 0.5
        # Ensure that we wait until the client is fully done.
        echo -n "Waiting"
        while [[ ! -f /tmp/python_rpc_done ]]; do
            echo -n "."
            sleep 0.1
        done
        echo
        # TODO(eric.cousineau): In script form, this generally works well (only
        # one interrupt needed); however, interactively we need a few more.
        while ps -p ${pid} > /dev/null; do
            kill -INT ${pid}
            sleep 0.05
        done
    fi
    py-check
}
sub-tests threading-loop

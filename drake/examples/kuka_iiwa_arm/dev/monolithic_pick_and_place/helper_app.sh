#!/bin/bash
set -x -e

export-prepend () 
{ 
    eval "export $1=\"$2:\$$1\""
}

env-extend () 
{ 
    local python_version=2.7;
    while [[ $# -gt 0 ]]; do
        case $1 in 
            -h | --help)
                echo "env-extend [--python-version <VERSION>] <PREFIX>";
                return
            ;;
            --python-version)
                python_version=$1;
                shift
            ;;
            *)
                break
            ;;
        esac;
    done;
    local prefix=${1%/};
    export-prepend PYTHONPATH $prefix/lib:$prefix/lib/python${python_version}/dist-packages;
    export-prepend PATH $prefix/bin;
    export-prepend LD_LIBRARY_PATH $prefix/lib;
    export-prepend PKG_CONFIG_PATH $prefix/lib/pkgconfig;
    echo "[ Environment extended: ${prefix} ]"
}

env-extend ${DRAKE}/build/install

# @see spartan:517bffd:apps/iiwa/runapp.sh (master)
drake-visualizer --script helper_app.py --bot-config helper_app.cfg "$@"

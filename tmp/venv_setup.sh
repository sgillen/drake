#!/bin/bash

_env_dir=$(cd $(dirname ${BASH_SOURCE}) && pwd)/venv
_python_ver_bin=python2
if [[ ! -f ${_env_dir}/bin/${_python_ver_bin} ]]; then
(
    set -eux
    cd $(dirname ${_env_dir})
    python_bin=$(which ${_python_ver_bin})
    ${python_bin} -m virtualenv --system-site-packages --python ${python_bin} ${_env_dir}
    set +eux
    source ${_env_dir}/bin/activate
    # Install some (if not all) needed dependencies.
    pip install -I -r ${_env_dir}/../venv_requirements.txt
    set -eux

    # Reflect system Python; make `python` and `python-config` fall through to
    # system, and use `python{major}-config`.
    rm ${_env_dir}/bin/python
    mv ${_env_dir}/bin/{python-config,${_python_ver_bin}-config}
    sed -i "s#/bin/python\b#/bin/${_python_ver_bin}#g" ${_env_dir}/bin/${_python_ver_bin}-config

    workspace=${_env_dir}/../..
    venv_python_bin=${_env_dir}/bin/${_python_ver_bin}
    cat > ${workspace}/user.bazelrc <<EOF
build --python_path=${venv_python_bin}
build --action_env=DRAKE_PYTHON_BIN_PATH=${venv_python_bin}
EOF
)
fi

source ${_env_dir}/bin/activate
unset _env_dir _python_ver_bin

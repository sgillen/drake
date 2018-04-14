#!/bin/bash
set -e -u -x

_install() {
    apt install --no-install-recommends -q "$@"
}

# Base build essentials
_install \
    build-essential git \
    make libssl-dev zlib1g-dev libbz2-dev libsqlite3-dev

# Python
_install \
    python2.7 python2.7-dev \
    python-pip python-setuptools python-wheel

# NumPy
_install \
    cython gfortran liblapack-dev libblas-dev

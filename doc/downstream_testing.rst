******************************************
Downstream Testing (Drake as a Dependency)
******************************************

Introduction
============

To ensure that Drake enables downstream consumption, there are downstream tests
that show basic usage of Drake as a dependency. Those tests are located in
the `Drake Shambhala <https://github.com/RobotLocomotion/drake-shambhala>`_
repository.

Continuous Integration
======================

Please see the Drake Shambhala `Continuous Integration <https://github.com/RobotLocomotion/drake-shambhala#continuous-integration>`_ section.

Local Testing
=============

If you are making build or API changes that may affect the downstream interface,
please try testing this locally on your system.

The following instructions are adapted from the Drake Shambhala
instructions located `here <https://github.com/RobotLocomotion/drake-shambhala/tree/master/drake_cmake_installed>`_, but are specific to your development
tree. This will use an existing source tree of Drake, build it with CMake
(to ensure a consistent compilation with a default CMake configuration), build
the downstream project, and then run its respective tests:

.. code-block:: shell

    # Build development version of Drake.
    cd drake  # Where you are developing.
    mkdir ../drake-build && cd ../drake-build  # Make sure this is a *fresh* folder.
    cmake ../drake
    # Build locally.
    make -j
    # Record the build's install directory.
    drake_install=${PWD}/install
    # Build Drake Shambhala using development version of Drake.
    cd ..
    git clone https://github.com/RobotLocomotion/drake-shambhala
    cd drake-shambhala/drake_cmake_installed
    # Follow 'Install Prerequisites' in the instructions linked above if you
    # have not already.
    mkdir build && cd build
    cmake -Ddrake_DIR=${drake_install}/lib/cmake/drake ..
    make -j  # We must build `all` before running `test`.
    make -j test

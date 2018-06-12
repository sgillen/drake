#!/bin/bash
set -x

set -e

bazel build --compiler=clang-4.0 -c dbg //systems/sensors:rgbd_renderer_ospray_test
# No error.
bazel-bin/systems/sensors/rgbd_renderer_ospray_test

bazel build --compiler=gcc-5 -c dbg //systems/sensors:rgbd_renderer_ospray_test
gdb -ex run bazel-bin/systems/sensors/rgbd_renderer_ospray_test

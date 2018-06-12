#!/bin/bash
set -x

set -e
bazel build -c dbg //systems/sensors:rgbd_renderer_ospray_test

set +e
gdb -ex run bazel-bin/systems/sensors/rgbd_renderer_ospray_test

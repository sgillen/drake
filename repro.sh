#!/bin/bash
set -x

bazel build -c dbg //systems/sensors:rgbd_renderer_ospray_test
gdb -ex run bazel-bin/systems/sensors/rgbd_renderer_ospray_test

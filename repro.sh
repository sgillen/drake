#!/bin/bash
set -e
(
    set -e
    for i in $(seq 100); do
        echo "[ $i ]"
        bazel run //systems/sensors:rgbd_renderer_ospray_test || exit 1
    done
)
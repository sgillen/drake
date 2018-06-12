#!/bin/bash
set -eux

cd $(dirname $0)
bazel run :ospray_path_tracer

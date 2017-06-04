# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_library",
)

package(
    default_visibility = ["//drake/examples/kuka_iiwa_arm:__subpackages__"],
)

drake_cc_library(
    name = "pick_and_place_state_machine",
    srcs = [
	"pick_and_place_state_machine.cc",
    ],
    hdrs = [
	"pick_and_place_state_machine.h",
    ],
    deps = [
        "//drake/common",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/common/trajectories:piecewise_quaternion",
        "//drake/examples/kuka_iiwa_arm/pick_and_place",
        "//drake/lcmtypes:schunk",
        "//drake/manipulation/planner:constraint_relaxing_ik",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "@bot_core_lcmtypes",
        "@robotlocomotion_lcmtypes",
    ],
)

drake_cc_binary(
    name = "pick_and_place_demo",
    srcs = [
        "pick_and_place_demo.cc",
    ],
    deps = [
        ":pick_and_place_state_machine",
        "//drake/examples/kuka_iiwa_arm/pick_and_place",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:iiwa_lcm",
        "//drake/util:lcm_util",
    ],
)

# === test/ ===

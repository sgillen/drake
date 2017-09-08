# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_library",
)
load("//tools:lint.bzl", "add_lint_tests")
#load(":pick_and_place_test.bzl", "pick_and_place_test")
#

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "push_and_pick_utils",
    srcs = [],
    hdrs = ["push_and_pick_utils.h"],
    deps = [
        "//drake/common",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/common/trajectories:piecewise_quaternion",
        "//drake/manipulation/planner:constraint_relaxing_ik",
    ],
)

drake_cc_library(
    name = "perception_base",
    hdrs = [
        "perception_base.h",
    ],
    deps = [
        "//drake/common",
        "//drake/systems/sensors:camera_info",
        "//drake/systems/sensors:image",
    ],
)

drake_cc_library(
    name = "push_and_pick_state_machine",
    srcs = [
        "push_and_pick_state_machine.cc",
        "push_and_pick_state_machine_system.cc",
    ],
    hdrs = [
        "push_and_pick_state_machine.h",
        "push_and_pick_state_machine_system.h",
    ],
    deps = [
        ":perception_base",
        ":push_and_pick_utils",
        "//drake/lcm",
        "//drake/lcmtypes:viewer",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm/pick_and_place",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/systems/framework:leaf_system",
        "//drake/systems/sensors:image",
        "//drake/systems/sensors:rgbd_camera",
        "//drake/systems/rendering:pose_vector",
    ],
)

drake_cc_binary(
    name = "push_pick_tree_tester",
    srcs = ["push_pick_tree_tester.cc"],
    add_test_rule = 1,
    data = [
        "//drake/examples/kuka_iiwa_arm:models",
        "//drake/manipulation/models/iiwa_description:models",
        "//drake/manipulation/models/wsg_50_description:models",
    ],
    test_rule_args = ["--num_configurations=1"],
    deps = [
        "//drake/common",
        "//drake/common:find_resource",
        "//drake/examples/kuka_iiwa_arm/pick_and_place",
        "//drake/lcm",
        "//drake/manipulation/util:simple_tree_visualizer",
        "//drake/manipulation/util:world_sim_tree_builder",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
        "//drake/util:lcm_util",
        "@com_github_gflags_gflags//:gflags",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
    ],
)

drake_cc_binary(
    name = "push_and_pick_demo",
    srcs = ["push_and_pick_demo.cc"],
    data = [
        "//drake/examples/kuka_iiwa_arm:models",
        "//drake/manipulation/models/iiwa_description:models",
        "//drake/manipulation/models/wsg_50_description:models",
    ],
    deps = [
        ":perception_base",
        ":push_and_pick_state_machine",
        "//drake/common:find_resource",
        "//drake/common:text_logging_gflags",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:iiwa_lcm",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:iiwa_wsg_diagram_factory",
        "//drake/lcm",
        "//drake/lcmtypes:iiwa",
        "//drake/manipulation/planner:robot_plan_interpolator",
        "//drake/manipulation/schunk_wsg:schunk_wsg_controller",
        "//drake/manipulation/sensors:xtion",
        "//drake/manipulation/util:world_sim_tree_builder",
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/systems/analysis:simulator",
        "@com_github_gflags_gflags//:gflags",
        "@lcmtypes_bot2_core",
    ],
)

add_lint_tests()

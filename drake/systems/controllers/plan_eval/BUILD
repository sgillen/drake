# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:gurobi.bzl", "gurobi_test_tags")
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "generic_plan",
    srcs = [
        "generic_plan.cc",
    ],
    hdrs = [
        "generic_plan.h",
    ],
    deps = [
        "//drake/manipulation/util:trajectory_utils",
        "//drake/multibody:rigid_body_tree_alias_groups",
        "//drake/systems/controllers:setpoint",
        "//drake/systems/controllers/qp_inverse_dynamics",
        "//drake/systems/controllers/qp_inverse_dynamics:param_parser",
        "//drake/systems/framework:value",
    ],
)

drake_cc_library(
    name = "manipulator_plans",
    srcs = [
        "manipulator_move_end_effector_plan.cc",
    ],
    hdrs = [
        "manipulator_move_end_effector_plan.h",
    ],
    deps = [
        ":generic_plan",
        "//drake/common:unused",
        "//drake/lcmtypes:manipulator_plan_move_end_effector",
        "//drake/util:lcm_util",
    ],
)

drake_cc_library(
    name = "test_common",
    hdrs = [
        "test/test_common.h",
    ],
    deps = [
        "//drake/multibody:rigid_body_tree_alias_groups",
        "//drake/systems/controllers/qp_inverse_dynamics:param_parser",
    ],
)

drake_cc_googletest(
    name = "manipulator_plan_test",
    data = [
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.alias_groups",  # noqa
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.id_controller_config",  # noqa
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":manipulator_plans",
        ":test_common",
        "//drake/common:find_resource",
        "//drake/common/test_utilities:eigen_matrix_compare",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_library(
    name = "humanoid_manipulation_plan",
    srcs = [
        "dev/humanoid_manipulation_plan.cc",
    ],
    hdrs = [
        "dev/humanoid_manipulation_plan.h",
    ],
    deps = [
        ":generic_plan",
        "//drake/manipulation/util:robot_state_msg_translator",
        "//drake/systems/controllers:zmp_planner",
        "//drake/systems/controllers/qp_inverse_dynamics:lcm_utils",
        "//drake/util",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
    ],
)

drake_cc_binary(
    name = "humanoid_manipulation_demo",
    srcs = [
        "dev/humanoid_manipulation_demo.cc",
    ],
    data = [
        "//drake/examples/valkyrie:models",
    ],
    deps = [
        "//drake/common:find_resource",
        "//drake/examples/valkyrie:valkyrie_constants",
        "//drake/manipulation/util:robot_state_msg_translator",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
    ],
)

drake_cc_library(
    name = "plan_eval_base_system",
    srcs = ["plan_eval_base_system.cc"],
    hdrs = ["plan_eval_base_system.h"],
    deps = [
        "//drake/multibody:rigid_body_tree_alias_groups",
        "//drake/systems/controllers:setpoint",
        "//drake/systems/controllers/qp_inverse_dynamics:param_parser",
        "//drake/systems/framework:leaf_system",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "generic_plan_test",
    data = [
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.alias_groups",  # noqa
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.id_controller_config",  # noqa
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":generic_plan",
        ":test_common",
        "//drake/common:find_resource",
        "//drake/common/test_utilities:eigen_matrix_compare",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

add_lint_tests()

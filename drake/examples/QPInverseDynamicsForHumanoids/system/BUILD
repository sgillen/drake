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

drake_cc_library(
    name = "manipulator_plan_eval_system",
    srcs = ["manipulator_plan_eval_system.cc"],
    hdrs = ["manipulator_plan_eval_system.h"],
    deps = [
        "//drake/lcmtypes:plan_eval_debug_info",
        "//drake/systems/controllers/plan_eval:plan_eval_base_system",
    ],
)

drake_cc_library(
    name = "manipulator_inverse_dynamics_controller",
    srcs = ["manipulator_inverse_dynamics_controller.cc"],
    hdrs = ["manipulator_inverse_dynamics_controller.h"],
    deps = [
        ":humanoid_status_translator_system",
        ":manipulator_plan_eval_system",
        "//drake/multibody/parsers",
        "//drake/systems/controllers:state_feedback_controller_interface",
        "//drake/systems/controllers/qp_inverse_dynamics:qp_inverse_dynamics_system",  # noqa
        "//drake/systems/controllers/qp_inverse_dynamics:qp_output_translator_system",  # noqa
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "manipulator_inverse_dynamics_controller_test",
    srcs = ["test/manipulator_inverse_dynamics_controller_test.cc"],
    data = [
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.alias_groups",  # noqa
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.id_controller_config",  # noqa
        "//drake/manipulation/models/iiwa_description:models",
    ],
    tags = gurobi_test_tags(),
    deps = [
        ":manipulator_inverse_dynamics_controller",
        "//drake/common:find_resource",
        "//drake/common/test_utilities:eigen_matrix_compare",
        "//drake/multibody/parsers",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/framework",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

drake_cc_googletest(
    name = "joint_level_controller_system_test",
    srcs = ["test/joint_level_controller_system_test.cc"],
    data = [
        "//drake/examples/valkyrie:models",
    ],
    deps = [
        ":atlas_joint_level_controller_system",
        "//drake/common:find_resource",
        "//drake/common/test_utilities:eigen_matrix_compare",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
    ],
)

#drake_cc_library(
#    name = "manipulator_plans",
#    srcs = [
#        "manipulator_move_end_effector_plan.cc",
#    ],
#    hdrs = [
#        "manipulator_move_end_effector_plan.h",
#    ],
#    deps = [
#        ":generic_plan",
#        "//drake/common:unused",
#        "//drake/lcmtypes:manipulator_plan_move_end_effector",
#        "//drake/util:lcm_util",
#    ],
#)

#drake_cc_googletest(
#    name = "manipulator_plan_test",
#    data = [
#        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.alias_groups",  # noqa
#        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.id_controller_config",  # noqa
#        "//drake/manipulation/models/iiwa_description:models",
#    ],
#    deps = [
#        ":manipulator_plans",
#        ":test_common",
#        "//drake/common:find_resource",
#        "//drake/common/test_utilities:eigen_matrix_compare",
#        "//drake/multibody:rigid_body_tree",
#        "//drake/multibody/parsers",
#    ],
#)

add_lint_tests()

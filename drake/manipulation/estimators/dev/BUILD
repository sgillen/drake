load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)

package(default_visibility = ["//visibility:public"])

"""
$ bazel query 'kind("cc_library", //drake/lcmtypes/...)' \
    | sort | sed -r 's#^(.*)$#"\1",#g'
"""
drake_cc_library(
    # Don't care to pick and choose
    name = "lcm_aggregate",
    srcs = [],
    hdrs = [],
    deps = [
        "//drake/lcmtypes:acrobot",
        "//drake/lcmtypes:automotive",
        "//drake/lcmtypes:body_acceleration",
        "//drake/lcmtypes:body_motion_data",
        "//drake/lcmtypes:body_wrench_data",
        "//drake/lcmtypes:constrained_values",
        "//drake/lcmtypes:contact_info_for_viz",
        "//drake/lcmtypes:contact_information",
        "//drake/lcmtypes:contact_results_for_viz",
        "//drake/lcmtypes:desired_body_motion",
        "//drake/lcmtypes:desired_centroidal_momentum_dot",
        "//drake/lcmtypes:desired_dof_motions",
        "//drake/lcmtypes:drake_signal",
        "//drake/lcmtypes:iiwa",
        "//drake/lcmtypes:inverse_dynamics_debug_info",
        "//drake/lcmtypes:joint_pd_override",
        "//drake/lcmtypes:manipulator_plan_move_end_effector",
        "//drake/lcmtypes:piecewise_polynomial",
        "//drake/lcmtypes:plan_eval_debug_info",
        "//drake/lcmtypes:polynomial",
        "//drake/lcmtypes:polynomial_matrix",
        "//drake/lcmtypes:qp_controller_input",
        "//drake/lcmtypes:qp_input",
        "//drake/lcmtypes:resolved_contact",
        "//drake/lcmtypes:schunk",
        "//drake/lcmtypes:support_data",
        "//drake/lcmtypes:viewer",
        "//drake/lcmtypes:whole_body_data",
        "//drake/lcmtypes:zmp_data",
    ],
)

drake_cc_library(
    name = "manipulation_tracker",
    includes = ["."],
    srcs = [
        "ManipulationTracker.cpp",
        "costs/AttachedApriltagCost.cpp",
        "costs/DynamicsCost.cpp",
        "costs/GelsightCost.cpp",
        "costs/JointStateCost.cpp",
        "costs/KinectFrameCost.cpp",
        "costs/NonpenetratingObjectCost.cpp",
        "costs/OptotrakMarkerCost.cpp",
        "costs/RobotStateCost.cpp",
    ],
    hdrs = [
        "common/sdf_2d_functions.hpp",
        "common/common.hpp",
        "ManipulationTracker.hpp",
        "costs/AttachedApriltagCost.hpp",
        "costs/DynamicsCost.hpp",
        "costs/GelsightCost.hpp",
        "costs/JointStateCost.hpp",
        "costs/KinectFrameCost.hpp",
        "costs/ManipulationTrackerCost.hpp",
        "costs/NonpenetratingObjectCost.hpp",
        "costs/OptotrakMarkerCost.hpp",
        "costs/RobotStateCost.hpp",
    ],
    deps = [
        "//drake/common",
        "//drake/multibody/joints",
        "//drake/lcm",
        ":lcm_aggregate",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
        "@yaml_cpp",
        "@bot_core_lcmtypes",
        # Do not exist yet
        "@bot_core_lcmgl:client",
        "@bot_frames",
    ],
)

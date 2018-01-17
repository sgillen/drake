# -*- python -*-

# Should include everything any consumer of Drake would ever need.
#
# Do not update this list by hand; instead, run build_components_refresh.py.
#
# When adding new components to the package, please also add the licenses for
# any new external dependencies to :external_licenses.
LIBDRAKE_COMPONENTS = [
    # "//drake/automotive/maliput/api:api",
    # "//drake/automotive/maliput/dragway:dragway",
    # "//drake/automotive/maliput/monolane:builder",
    # "//drake/automotive/maliput/monolane:lanes",
    # "//drake/automotive/maliput/monolane:loader",
    # "//drake/automotive/maliput/monolane:monolane",
    # "//drake/automotive/maliput/multilane:builder",
    # "//drake/automotive/maliput/multilane:lanes",
    # "//drake/automotive/maliput/multilane:loader",
    # "//drake/automotive/maliput/multilane:multilane",
    # "//drake/automotive/maliput/rndf:builder",
    # "//drake/automotive/maliput/rndf:lanes",
    # "//drake/automotive/maliput/rndf:loader",
    # "//drake/automotive/maliput/rndf:rndf",
    # "//drake/automotive/maliput/utility:utility",
    # "//drake/automotive:automotive_simulator",
    # "//drake/automotive:bicycle_car",
    # "//drake/automotive:box_car_vis",
    # "//drake/automotive:car_vis",
    # "//drake/automotive:car_vis_applicator",
    # "//drake/automotive:curve2",
    # "//drake/automotive:generated_translators",
    # "//drake/automotive:generated_vectors",
    # "//drake/automotive:idm_controller",
    # "//drake/automotive:idm_planner",
    # "//drake/automotive:lane_direction",
    # "//drake/automotive:maliput_railcar",
    # "//drake/automotive:mobil_planner",
    # "//drake/automotive:monolane_onramp_merge",
    # "//drake/automotive:prius_vis",
    # "//drake/automotive:pure_pursuit",
    # "//drake/automotive:pure_pursuit_controller",
    # "//drake/automotive:road_odometry",
    # "//drake/automotive:simple_car",
    # "//drake/automotive:simple_powertrain",
    # "//drake/automotive:trajectory_car",
    "//drake/common/proto:call_matlab",
    "//drake/common/proto:call_python",
    "//drake/common/proto:matlab_rpc",
    "//drake/common/proto:protobuf",
    "//drake/common/trajectories/qp_spline:continuity_constraint",
    "//drake/common/trajectories/qp_spline:spline_generation",
    "//drake/common/trajectories/qp_spline:spline_information",
    "//drake/common/trajectories/qp_spline:value_constraint",
    "//drake/common/trajectories:piecewise_function",
    "//drake/common/trajectories:piecewise_polynomial",
    "//drake/common/trajectories:piecewise_polynomial_trajectory",
    "//drake/common/trajectories:piecewise_quaternion",
    "//drake/common/trajectories:trajectory",
    "//drake/common:autodiff",
    "//drake/common:common",
    "//drake/common:cond",
    "//drake/common:copyable_unique_ptr",
    "//drake/common:default_scalars",
    "//drake/common:double",
    "//drake/common:drake_path",
    "//drake/common:dummy_value",
    "//drake/common:essential",
    "//drake/common:extract_double",
    "//drake/common:find_resource",
    "//drake/common:hash",
    "//drake/common:is_approx_equal_abstol",
    "//drake/common:is_cloneable",
    "//drake/common:nice_type_name",
    "//drake/common:number_traits",
    "//drake/common:polynomial",
    "//drake/common:reinit_after_move",
    "//drake/common:scoped_singleton",
    "//drake/common:sorted_vectors_have_intersection",
    "//drake/common:symbolic",
    "//drake/common:symbolic_decompose",
    "//drake/common:text_logging_gflags_h",
    "//drake/common:type_safe_index",
    "//drake/common:unused",
    # "//drake/geometry/query_results:penetration_as_point_pair",
    # "//drake/geometry:frame_kinematics",
    # "//drake/geometry:geometry_context",
    # "//drake/geometry:geometry_frame",
    # "//drake/geometry:geometry_ids",
    # "//drake/geometry:geometry_index",
    # "//drake/geometry:geometry_instance",
    # "//drake/geometry:geometry_state",
    # "//drake/geometry:geometry_system",
    # "//drake/geometry:geometry_visualization",
    # "//drake/geometry:identifier",
    # "//drake/geometry:internal_frame",
    # "//drake/geometry:internal_geometry",
    # "//drake/geometry:shape_specification",
    "//drake/lcm:interface",
    "//drake/lcm:lcm",
    "//drake/lcm:lcm_log",
    "//drake/lcm:mock",
    "//drake/lcm:translator_base",
    # "//drake/manipulation/planner:constraint_relaxing_ik",
    # "//drake/manipulation/planner:robot_plan_interpolator",
    # "//drake/manipulation/schunk_wsg:schunk_wsg_constants",
    # "//drake/manipulation/schunk_wsg:schunk_wsg_controller",
    # "//drake/manipulation/schunk_wsg:schunk_wsg_lcm",
    # "//drake/manipulation/util:frame_pose_tracker",
    # "//drake/manipulation/util:moving_average_filter",
    # "//drake/manipulation/util:robot_state_msg_translator",
    # "//drake/manipulation/util:sim_diagram_builder",
    # "//drake/manipulation/util:simple_tree_visualizer",
    # "//drake/manipulation/util:trajectory_utils",
    # "//drake/manipulation/util:world_sim_tree_builder",
    "//drake/math:autodiff",
    "//drake/math:continuous_algebraic_riccati_equation",
    "//drake/math:discrete_algebraic_riccati_equation",
    "//drake/math:eigen_sparse_triplet",
    "//drake/math:evenly_distributed_pts_on_sphere",
    "//drake/math:expmap",
    "//drake/math:geometric_transform",
    "//drake/math:gradient",
    "//drake/math:gray_code",
    "//drake/math:jacobian",
    "//drake/math:matrix_util",
    "//drake/math:orthonormal_basis",
    "//drake/math:quadratic_form",
    "//drake/math:saturate",
    "//drake/math:vector3_util",
    # "//drake/multibody/benchmarks/acrobot:acrobot",
    # "//drake/multibody/benchmarks/free_body:free_body",
    # "//drake/multibody/benchmarks/mass_damper_spring:mass_damper_spring_analytical_solution",  # noqa
    # "//drake/multibody/collision:bullet_collision",
    # "//drake/multibody/collision:collision",
    # "//drake/multibody/collision:collision_api",
    # "//drake/multibody/collision:fcl_collision",
    # "//drake/multibody/collision:model",
    # "//drake/multibody/constraint:constraint",
    # "//drake/multibody/constraint:constraint_solver",
    # "//drake/multibody/joints:joints",
    # "//drake/multibody/multibody_tree/math:spatial_acceleration",
    # "//drake/multibody/multibody_tree/math:spatial_algebra",
    # "//drake/multibody/multibody_tree/math:spatial_force",
    # "//drake/multibody/multibody_tree/math:spatial_vector",
    # "//drake/multibody/multibody_tree/math:spatial_velocity",
    # "//drake/multibody/multibody_tree:multibody_tree",
    # "//drake/multibody/multibody_tree:multibody_tree_context",
    # "//drake/multibody/multibody_tree:multibody_tree_element",
    # "//drake/multibody/multibody_tree:multibody_tree_indexes",
    # "//drake/multibody/multibody_tree:multibody_tree_topology",
    # "//drake/multibody/multibody_tree:rotational_inertia",
    # "//drake/multibody/multibody_tree:spatial_inertia",
    # "//drake/multibody/multibody_tree:unit_inertia",
    # "//drake/multibody/parsers:parsers",
    # "//drake/multibody/rigid_body_plant:compliant_contact_model",
    # "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
    # "//drake/multibody/rigid_body_plant:create_load_robot_message",
    # "//drake/multibody/rigid_body_plant:drake_visualizer",
    # "//drake/multibody/rigid_body_plant:frame_visualizer",
    # "//drake/multibody/rigid_body_plant:rigid_body_plant",
    # "//drake/multibody/rigid_body_plant:rigid_body_plant_that_publishes_xdot",
    # "//drake/multibody/shapes:shapes",
    # "//drake/multibody:approximate_ik",
    # "//drake/multibody:global_inverse_kinematics",
    # "//drake/multibody:inverse_kinematics",
    # "//drake/multibody:kinematics_cache",
    # "//drake/multibody:rigid_body",
    # "//drake/multibody:rigid_body_actuator",
    # "//drake/multibody:rigid_body_constraint",
    # "//drake/multibody:rigid_body_frame",
    # "//drake/multibody:rigid_body_loop",
    # "//drake/multibody:rigid_body_tree",
    # "//drake/multibody:rigid_body_tree_alias_groups",
    # "//drake/multibody:rigid_body_tree_alias_groups_proto",
    # "//drake/multibody:rigid_body_tree_construction",
    # "//drake/perception:point_cloud",
    # "//drake/perception:point_cloud_flags",
    # "//drake/solvers:bilinear_product_util",
    # "//drake/solvers:binding",
    # "//drake/solvers:constraint",
    # "//drake/solvers:cost",
    # "//drake/solvers:create_constraint",
    # "//drake/solvers:create_cost",
    # "//drake/solvers:decision_variable",
    # "//drake/solvers:dreal_solver",
    # "//drake/solvers:equality_constrained_qp_solver",
    # "//drake/solvers:evaluator_base",
    # "//drake/solvers:function",
    # "//drake/solvers:gurobi_qp",
    # "//drake/solvers:gurobi_solver",
    # "//drake/solvers:indeterminate",
    # "//drake/solvers:integer_optimization_util",
    # "//drake/solvers:ipopt_solver",
    # "//drake/solvers:linear_system_solver",
    # "//drake/solvers:mathematical_program",
    # "//drake/solvers:mathematical_program_api",
    # "//drake/solvers:mixed_integer_optimization_util",
    # "//drake/solvers:moby_lcp_solver",
    # "//drake/solvers:mosek_solver",
    # "//drake/solvers:nlopt_solver",
    # "//drake/solvers:non_convex_optimization_util",
    # "//drake/solvers:rotation_constraint",
    # "//drake/solvers:snopt_solver",
    # "//drake/solvers:solver_id",
    # "//drake/solvers:solver_type",
    # "//drake/solvers:solver_type_converter",
    # "//drake/solvers:symbolic_extraction",
    # "//drake/solvers:system_identification",
    # "//drake/systems/analysis:analysis",
    # "//drake/systems/analysis:explicit_euler_integrator",
    # "//drake/systems/analysis:implicit_euler_integrator",
    # "//drake/systems/analysis:integrator_base",
    # "//drake/systems/analysis:runge_kutta2_integrator",
    # "//drake/systems/analysis:runge_kutta3_integrator",
    # "//drake/systems/analysis:semi_explicit_euler_integrator",
    # "//drake/systems/analysis:simulator",
    # "//drake/systems/controllers/plan_eval:generic_plan",
    # "//drake/systems/controllers/plan_eval:plan_eval_base_system",
    # "//drake/systems/controllers/qp_inverse_dynamics:id_controller_config",
    # "//drake/systems/controllers/qp_inverse_dynamics:lcm_utils",
    # "//drake/systems/controllers/qp_inverse_dynamics:param_parser",
    # "//drake/systems/controllers/qp_inverse_dynamics:qp_inverse_dynamics",
    # "//drake/systems/controllers/qp_inverse_dynamics:qp_inverse_dynamics_system",  # noqa
    # "//drake/systems/controllers/qp_inverse_dynamics:qp_output_translator_system",  # noqa
    # "//drake/systems/controllers/qp_inverse_dynamics:robot_kinematic_state",
    # "//drake/systems/controllers/qp_inverse_dynamics:robot_kinematic_state_translator_system",  # noqa
    # "//drake/systems/controllers:control_util",
    # "//drake/systems/controllers:instantaneous_qp_controller",
    # "//drake/systems/controllers:inverse_dynamics",
    # "//drake/systems/controllers:inverse_dynamics_controller",
    # "//drake/systems/controllers:linear_model_predictive_controller",
    # "//drake/systems/controllers:linear_quadratic_regulator",
    # "//drake/systems/controllers:pid_controlled_system",
    # "//drake/systems/controllers:pid_controller",
    # "//drake/systems/controllers:qp_common",
    # "//drake/systems/controllers:setpoint",
    # "//drake/systems/controllers:side",
    # "//drake/systems/controllers:state_feedback_controller_interface",
    # "//drake/systems/controllers:zmp_planner",
    # "//drake/systems/controllers:zmp_util",
    # "//drake/systems/estimators:kalman_filter",
    # "//drake/systems/estimators:luenberger_observer",
    # "//drake/systems/framework:abstract_values",
    # "//drake/systems/framework:cache",
    # "//drake/systems/framework:context",
    # "//drake/systems/framework:continuous_state",
    # "//drake/systems/framework:diagram",
    # "//drake/systems/framework:diagram_builder",
    # "//drake/systems/framework:diagram_context",
    # "//drake/systems/framework:diagram_continuous_state",
    # "//drake/systems/framework:discrete_values",
    # "//drake/systems/framework:event_collection",
    # "//drake/systems/framework:framework",
    # "//drake/systems/framework:framework_common",
    # "//drake/systems/framework:input_port_descriptor",
    # "//drake/systems/framework:input_port_evaluator_interface",
    # "//drake/systems/framework:input_port_value",
    # "//drake/systems/framework:leaf_context",
    # "//drake/systems/framework:leaf_output_port",
    # "//drake/systems/framework:leaf_system",
    # "//drake/systems/framework:output_port_listener_interface",
    # "//drake/systems/framework:output_port_value",
    # "//drake/systems/framework:parameters",
    # "//drake/systems/framework:single_output_vector_source",
    # "//drake/systems/framework:state",
    # "//drake/systems/framework:system",
    # "//drake/systems/framework:system_constraint",
    # "//drake/systems/framework:system_scalar_converter",
    # "//drake/systems/framework:system_symbolic_inspector",
    # "//drake/systems/framework:value",
    # "//drake/systems/framework:vector",
    # "//drake/systems/framework:vector_system",
    # "//drake/systems/lcm:lcm",
    # "//drake/systems/lcm:lcm_driven_loop",
    # "//drake/systems/lcm:lcmt_drake_signal_translator",
    # "//drake/systems/lcm:translator",
    # "//drake/systems/lcm:translator_system",
    # "//drake/systems/plants/spring_mass_system:spring_mass_system",
    # "//drake/systems/primitives:adder",
    # "//drake/systems/primitives:affine_system",
    # "//drake/systems/primitives:constant_value_source",
    # "//drake/systems/primitives:constant_vector_source",
    # "//drake/systems/primitives:demultiplexer",
    # "//drake/systems/primitives:first_order_low_pass_filter",
    # "//drake/systems/primitives:gain",
    # "//drake/systems/primitives:integrator",
    # "//drake/systems/primitives:linear_system",
    # "//drake/systems/primitives:matrix_gain",
    # "//drake/systems/primitives:multiplexer",
    # "//drake/systems/primitives:pass_through",
    # "//drake/systems/primitives:piecewise_polynomial_affine_system",
    # "//drake/systems/primitives:piecewise_polynomial_linear_system",
    # "//drake/systems/primitives:primitives",
    # "//drake/systems/primitives:random_source",
    # "//drake/systems/primitives:saturation",
    # "//drake/systems/primitives:signal_log",
    # "//drake/systems/primitives:signal_logger",
    # "//drake/systems/primitives:time_varying_data",
    # "//drake/systems/primitives:trajectory_source",
    # "//drake/systems/primitives:zero_order_hold",
    # "//drake/systems/rendering:drake_visualizer_client",
    # "//drake/systems/rendering:frame_velocity",
    # "//drake/systems/rendering:pose_aggregator",
    # "//drake/systems/rendering:pose_bundle",
    # "//drake/systems/rendering:pose_bundle_to_draw_message",
    # "//drake/systems/rendering:pose_stamped_t_pose_vector_translator",
    # "//drake/systems/rendering:pose_vector",
    # "//drake/systems/robotInterfaces:body_motion_data",
    # "//drake/systems/robotInterfaces:qp_locomotion_plan",
    # "//drake/systems/sensors:accelerometer",
    # "//drake/systems/sensors:beam_model",
    # "//drake/systems/sensors:beam_model_params",
    # "//drake/systems/sensors:camera_info",
    # "//drake/systems/sensors:depth_sensor",
    # "//drake/systems/sensors:depth_sensor_to_lcm_point_cloud_message",
    # "//drake/systems/sensors:gyroscope",
    # "//drake/systems/sensors:image",
    # "//drake/systems/sensors:image_to_lcm_image_array_t",
    # "//drake/systems/sensors:optitrack_encoder",
    # "//drake/systems/sensors:optitrack_sender",
    # "//drake/systems/sensors:rgbd_camera",
    # "//drake/systems/sensors:rgbd_renderer",
    # "//drake/systems/sensors:rotary_encoders",
    # "//drake/systems/sensors:sensors",
    # "//drake/systems/trajectory_optimization:direct_collocation",
    # "//drake/systems/trajectory_optimization:direct_transcription",
    # "//drake/systems/trajectory_optimization:multiple_shooting",
    "//drake/util:lcm_util",
    "//drake/util:util",
]

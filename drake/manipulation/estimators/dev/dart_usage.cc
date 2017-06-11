#include "dart.h"

int do_main() {
  // Keep name mangling outside of Dart estimation???
  DartScene scene(tree, ic);

  // Will contain optimization-relevant things, but will not give insight into
  // peers.
  DartFormulation::Param formulation_param {
    .estimated_joints = estimated_joints,
  };
  DartFormulation formulation(scene, formulation_param);

  DartEstimator estimator(formulation);
  // weights must coincide with scene's stuff

  DartJointObjective::Param joint_param = {
    // Must coincide with subset of joints
    .estimated_weights = estimated_weights,
  };
  auto* joint =
    estimator.AddObjective(new DartJointObjective(formulation, joint_param));

  DartDepthImageIcpObjective::Param depth_param {
    .camera_frame = camera_frame,
    .icp_variance = 20,
    .free_space_variance = 0,
    .downsample_factor = 5,
    .point_cloud_bounds = {
        .x = {-2, 2},
        .y = {-2, 2},
        .z = {-2, 2},
    },
    .debug {
      .use_lcmgl = true,
    },
  };
  auto* depth =
    estimator.AddObjective(
        new DartDepthImageIcpObjective(formulation, depth_param));

  // DartNonpenetrationObjective::Param nonpen_param {
  //   .variance = 0.001,  // m
  //   .cliques {
  //     {table_id, target_id},
  //   },
  //   .debug {
  //     .use_lcmgl = true,
  //   },
  // };
  // auto* nonpen =
  //   estimator.AddObjective(
  //       new DartNonpenetrationObjective(formulation, nonpen_param));

  // Observations
  estimator.ObserveTime(t);
  joint->ObserveState(t, q, v);
  depth->ObserveImage(t, depth_image);
  // nonpen does not need observations, as it relies on priors.

  // Update estimator
  // - will check the timestamp of each objective's observations.
  tie(q, v) = estimator.Update();

  // NOTE: How to handle generalized multi-rate estimation?
  // Seems like the output will still be non-smooth.

  template <typename T>
  auto ComputeVarianceWeight(const T& variance) {
    // NOTE: Test with ArrayXd.
    return 1 / (2 * pow(variance, 2));
  }

  // In objectives, when addings costs
  Objective {
    Init() {
      e0 = Vector3d::Zeros();
      Je0_qopt_0 = MatrixXd::Zeros(scene->get_num_positions_estimated());

      position_vars = formulation->GetPositionVars();
      // This is the cumulative cost.
      icp_cost_ =
          prog().AddCost(MakeQuadraticCost(IcpErrorNorm::Zero(), IcpErrorposition_vars)).constraint().get();
      free_space_cost_ =
          prog().AddCost(MakeQuadraticCost(...)).constraint().get();
    }
    Update() {
      icp_weight = ComputeVarianceWeight(param_.icp_variance);
      for (error : errors) {
        icp_error_accumulator_.AddL2ErrorCost(e, Je_qopt);
      }
      icp_error_accumulator_.UpdateCoefficients(
          icp_cost_,
          icp_weight,
          formulation_->kinematics_slice.q.indices());
    }
    formulation_;
    // Accumulate L2 norms in full joint space
    icp_error_accumulator_;
    // Render L2 Norm in proper decision variables
    QuadraticCost* icp_cost_{};
    QuadraticCost* free_space_cost_{};
  }
}

int main() {
  return do_main();
}

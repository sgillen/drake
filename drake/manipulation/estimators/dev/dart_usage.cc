#include "dart.h"

int main() {
  // Keep name mangling outside of Dart estimation???
  DartScene scene(tree, ic);

  // Will contain optimization-relevant things, but will not give insight into
  // peers.
  DartFormulation formulation(scene, estimated_joints);

  DartEstimator estimator(formulation);
  // weights must coincide with scene's stuff

  DartJointObjective::Param joint_param = {
    .estimated_weights = estimated_weights,
  };
  auto* joint =
    estimator.AddObjective(new DartJointObjective(formulation, joint_param));

  DartDepthImageIcpObjective::Param depth_param {
    .frame = camera_frame,
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
  joint->ObserveKinematicState(t, q, v);
  depth->ObserveImage(t, depth_image);
  // nonpen does not need observations, as it relies on priors.

  // Update estimator
  // - will check the timestamp of each objective's observations.
  tie(q, v) = estimator.Update();

  // NOTE: How to handle generalized multi-rate estimation?
  // Seems like the output will still be non-smooth.


  // In objectives, when addings costs
  Objective {
    Init() {
      e0 = Vector3d::Zeros();
      Je0_qopt_0 = MatrixXd::Zeros(scene->get_num_positions_estimated());

      position_vars = formulation->GetPositionVars();
      // This is the cumulative cost.
      icp_error_norm_ =
          prog().AddCost(MakeQuadraticCost(IcpErrorNorm::Zero(), IcpErrorposition_vars)).constraint();
    }
    Update() {
      for (error : errors) {
        icp_error_accumulator_.AddL2ErrorCost(e, Je_qopt);
      }
      icp_error_accumulator_.RenderCost(
          icp_error_sum_,
          formulation_->kinematics_slice.q.indices());
    }
    formulation_;
    // Accumulate L2 norms in full joint space
    icp_error_accumulator_;
    // Render L2 Norm in proper decision variables
    QuadraticCost icp_error_sum_;
  }
}

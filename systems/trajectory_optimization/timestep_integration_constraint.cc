#include "drake/systems/trajectory_optimization/timestep_integration_constraint.h"

namespace drake{
namespace systems{
namespace trajectory_optimization{

TimestepIntegrationConstraint::TimestepIntegrationConstraint(
    const RigidBodyTree<double>& tree,
    std::shared_ptr<plants::KinematicsCacheWithVHelper<AutoDiffXd>>
        kinematics_cache_with_v_helper, int num_lambda) // constraints: q; v+; v-; lambda
    : solvers::Constraint(num_lambda, num_lambda+2*tree.get_num_velocities() + tree.get_num_positions(),
        Eigen::MatrixXd::Zero(num_lambda, 1),
        Eigen::MatrixXd::Zero(num_lambda, 1)), 
      tree_(&tree),
      num_positions_{tree.get_num_positions()},
      num_velocities_{2*tree.get_num_velocities()},
      num_lambda_{num_lambda},
      kinematics_cache_with_v_helper_{kinematics_cache_with_v_helper}{}

void TimestepIntegrationConstraint::DoEval(
  const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const {
  AutoDiffVecXd y_t;
  DoEval(math::initializeAutoDiff(x), y_t);
  y = math::autoDiffToValueMatrix(y_t);
}

void TimestepIntegrationConstraint::DoEval(
  const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd& y) const {

  int x_count = 0;

  auto x_segment = [x, &x_count](int num_element) {
    x_count += num_element;
    return x.segment(x_count - num_element, num_element);
  };

  const AutoDiffVecXd q = x_segment(num_positions_);
  const AutoDiffVecXd v_plus = x_segment(num_velocities_/2);
  const AutoDiffVecXd v_minus = x_segment(num_velocities_/2);
  const AutoDiffVecXd lambda = x_segment(num_lambda_);

  auto kinsol = kinematics_cache_with_v_helper_->UpdateKinematics(q, v_minus);

  const auto M = tree_->massMatrix(kinsol);

  const MatrixX<AutoDiffXd> map_v_to_qdot =
      RigidBodyTree<double>::GetVelocityToQDotMapping(kinsol);
  
  // rigid_body_tree.cc should output: "did the assignmnt" 3 times here
  const AutoDiffVecXd qd_plus = map_v_to_qdot * v_plus;
  const AutoDiffVecXd qd_minus = map_v_to_qdot * v_minus;

  RigidBody<double>* brick = tree_->FindBody("contact_implicit_brick");
  RigidBody<double>*ww = tree_->FindBody("world");
//  int body_ind = tree_->FindBodyIndex("contact_implicit_brick");
  const auto& contact_points_a = brick->get_contact_points();
  const auto& contact_points_b = ww->get_contact_points();
  Eigen::VectorXi idx_a(1), idx_b(1);
  idx_a << 1;
  idx_b << 0;
  Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>, Eigen::Dynamic, Eigen::Dynamic> J;
  tree_->computeContactJacobians(kinsol, idx_a, idx_b, contact_points_a, contact_points_b, J);
  
  // Jqdot_plus_l - Jqdot_minus_l = -(1+restitution)J*M^(-1)*J^T*lambda
  y = J*qd_plus - J*qd_minus + J*M.inverse()*J.transpose()*lambda;
}

} // namespace trajectory_optimization
} // namespace systems
} // namespace drake
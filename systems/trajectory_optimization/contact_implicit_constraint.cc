#include "drake/systems/trajectory_optimization/contact_implicit_constraint.h"

#include "drake/multibody/kinematics_cache.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

void CollisionStuff(
    const RigidBodyTree<double>& tree_,
    const RigidBodyTree<double>& empty_tree_,
    const VectorX<AutoDiffXd>& q,
    const VectorX<AutoDiffXd>& v,
    const MatrixX<AutoDiffXd>& lambda_c,
    VectorX<AutoDiffXd>* phi_out,
    MatrixX<AutoDiffXd>* Jphi_out,
    MatrixX<double>* T_normals) {

  RigidBody<double>* brick = tree_->FindBody("contact_implicit_brick");
  const auto& contact_points_a = brick->get_contact_points();
  const KinematicsCache<double> cache = tree_->doKinematics(
        math::autoDiffToValueMatrix(q), math::autoDiffToValueMatrix(v));

  const auto points = tree_->transformPoints(
      cache, contact_points_a, brick->get_body_index(), 0);

  Eigen::VectorXd phi;
  Eigen::Matrix3Xd normal, contact_points_b, body_x;
  std::vector<int> body_idx;

  // TODO: Just project other stuff away?
  const KinematicsCache<double> scene_cache = empty_tree_->doKinematics(
      empty_tree_->getZeroConfiguration());

  const_cast<RigidBodyTree<double>*>(empty_tree_)->collisionDetectFromPoints(
      scene_cache, points,
      phi, normal, contact_points_b, body_x, body_idx, false);

  // TODO: Need to remap body indices.

  Eigen::VectorXi idx_a(1), idx_b(body_idx.size());
  idx_a << tree_->FindBodyIndex("contact_implicit_brick");
  for (size_t i = 0; i < body_idx.size(); i++) {
    idx_b[i] = body_idx[i];
  }

  MatrixX<AutoDiffXd> J;
  tree_->computeContactJacobians(kinsol, idx_a, idx_b, contact_points_a, contact_points_b, J);

  MatrixX<AutoDiffXd> tensornormal =
        Eigen::MatrixXd::Zero(num_lambda_, num_contacts_);
  for (int i = 0; i < num_contacts_; i++) {
    for (int j = 0; j < 3; j++) {
      tensornormal(3*i + j, i) = normal(j, i);
    }
  }
  VectorX<AutoDiffXd> autophi(num_contacts_);
  for (int i = 0; i < num_contacts_; i++) {
    autophi[i] = phi[i];
  }

  auto normallambda = tensornormal.transpose()*lambda;

  using namespace std;

  cout << "q: " << q.transpose() << endl;
  cout << "phi: " << phi.transpose() << endl;
  cout << "contact_points_a:\n" << contact_points_a << "\n---\n";
  cout << "points:\n" << points << "\n---\n";
  cout << "contact_points_b:\n" << contact_points_b << "\n---\n";

}

ContactImplicitConstraint::ContactImplicitConstraint(
    const RigidBodyTree<double>& tree,
    std::shared_ptr<plants::KinematicsCacheWithVHelper<AutoDiffXd>>
        kinematics_cache_with_v_helper, int num_lambda, double tol)
    : Constraint(num_lambda/3 + 1, tree.get_num_positions() +
                  2*tree.get_num_velocities() + num_lambda,
        Eigen::MatrixXd::Zero(num_lambda/3 + 1, 1),
        Eigen::MatrixXd::Constant(num_lambda/3 + 1, 1, tol)), 
      tree_(&tree),
      num_positions_{tree.get_num_positions()},
      num_velocities_{2*tree.get_num_velocities()},
      num_lambda_{num_lambda},
      num_contacts_{num_lambda/3},
      tol_{tol},
      kinematics_cache_with_v_helper_{kinematics_cache_with_v_helper}{
          Eigen::VectorXd UB(num_contacts_ + 1, 1);
          UB.head(num_contacts_) = Eigen::MatrixXd::Constant(
                      num_contacts_, 1, std::numeric_limits<double>::infinity());
          UB.tail(1) = Eigen::MatrixXd::Constant(1, 1, tol_);
          UpdateUpperBound(UB);
      }

void ContactImplicitConstraint::DoEval(
  const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), y_t);
  y = math::autoDiffToValueMatrix(y_t);
}

void ContactImplicitConstraint::DoEval(
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

  MatrixX<double> tensornormal;
  CollisionStuff(
    tree_, q, v_minus, lambda,
    &phi, &Jphi, &tensornormal);

  auto lambda_phi = tensornormal.transpose()*lambda;

  y << lambda_phi, 
        phi.transpose()*lambda_phi;
}

TimestepIntegrationConstraint::TimestepIntegrationConstraint(
    const RigidBodyTree<double>& tree,
    std::shared_ptr<plants::KinematicsCacheWithVHelper<AutoDiffXd>>
        kinematics_cache_with_v_helper, int num_lambda, double elasticity) // constraints: q; v+; v-; lambda
    : solvers::Constraint(num_lambda/3, num_lambda+2*tree.get_num_velocities() + tree.get_num_positions(),
        Eigen::MatrixXd::Zero(num_lambda/3, 1),
        Eigen::MatrixXd::Zero(num_lambda/3, 1)), 
      tree_(&tree),
      num_positions_{tree.get_num_positions()},
      num_velocities_{2*tree.get_num_velocities()},
      num_lambda_{num_lambda},
      num_contacts_{num_lambda/3},
      elasticity_{elasticity},
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
  
  const AutoDiffVecXd qd_plus = map_v_to_qdot * v_plus;
  const AutoDiffVecXd qd_minus = map_v_to_qdot * v_minus;

  VectorX<AutoDiffXd> phi;
  VectorX<AutoDiffXd> Jphi;
  MatrixX<double> tensornormal;
  CollisionStuff(
    tree_, q, v_minus, lambda,
    &phi, &Jphi, &tensornormal);

  auto lambda_phi = tensornormal.transpose()*lambda;

  // Jqdot_plus_l - Jqdot_minus_l = -(1+restitution)J*M^(-1)*J^T*lambda
  y = Jphi*qd_plus - Jphi*qd_minus -
          (1+elasticity_)*Jphi*M.inverse()*Jphi.transpose()*lambda_phi;
}


}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
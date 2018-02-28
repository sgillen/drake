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
    const MatrixX<AutoDiffXd>& lambda,
    VectorX<AutoDiffXd>* phi_out,
    MatrixX<AutoDiffXd>* Jphi_out,
    MatrixX<double>* tensornormal_out) {

  auto kinsol = tree_->doKinematics(q, v);

  RigidBody<double>* brick = tree_->FindBody("contact_implicit_brick");
  const auto& contacts_B = brick->get_contact_points();
  const KinematicsCache<double> cache = tree_->doKinematics(
        math::autoDiffToValueMatrix(q), math::autoDiffToValueMatrix(v));

  const int num_lambda_ = lambda.rows();
  const int num_contacts_ = num_lambda_ / 3;

  // `B` is body frame. `W` is world frame.
  const auto contacts_W = tree_->transformPoints(
      cache, contacts_B, brick->get_body_index(), 0 /* world */);

  Eigen::VectorXd phi;
  // `L` is the local frame of whatever bodies in the world.
  Eigen::Matrix3Xd normal, world_contacts_W, world_contacts_L;
  std::vector<int> body_idx;

  // Do collision without body in the way, so that points are not attracted to
  // it.
  // TODO: Just project other stuff away?
  const KinematicsCache<double> scene_cache = empty_tree_->doKinematics(
      empty_tree_->getZeroConfiguration());
  const_cast<RigidBodyTree<double>*>(empty_tree_)->collisionDetectFromPoints(
      scene_cache, contacts_W,
      phi, normal, world_contacts_W, world_contacts_L, body_idx, false);

  // TODO: Need to remap body indices.
  Eigen::VectorXi idx_a(1), idx_b(body_idx.size());
  idx_a << tree_->FindBodyIndex("contact_implicit_brick");
  for (size_t i = 0; i < body_idx.size(); i++) {
    idx_b[i] = body_idx[i];
  }

  // `J` is 3*nc x nq. Projecting along normals yields Jphi.
  MatrixX<AutoDiffXd> J;
  tree_->computeContactJacobians(
      kinsol, idx_a, idx_b, contacts_B, world_contacts_W, J);

  // Project along normals.
  MatrixX<double>& tensornormal = *tensornormal_out;
  tensornormal = Eigen::MatrixXd::Zero(num_lambda_, num_contacts_);
  for (int i = 0; i < num_contacts_; i++) {
    for (int j = 0; j < 3; j++) {
      tensornormal(3*i + j, i) = normal(j, i);
    }
  }

  *Jphi_out = tensornormal.transpose() * J;
  *phi_out = math::initializeAutoDiffGivenGradientMatrix(
      phi, math::autoDiffToValueMatrix(*Jphi_out));

  using namespace std;

  cout << "q: " << q.transpose() << endl;
  cout << "phi: " << phi.transpose() << endl;
  cout << "contacts_B:\n" << contacts_B << "\n---\n";
  cout << "contacts_W:\n" << contacts_W << "\n---\n";
  cout << "world_contacts_W:\n" << world_contacts_W << "\n---\n";
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
#include "drake/systems/trajectory_optimization/elastic_contact_implicit_direct_transcription.h"

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/trajectory_optimization/rigid_body_tree_multiple_shooting_internal.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
using plants::KinematicsCacheWithVHelper;

/**
 * Implements the constraint for the backward Euler integration
 * <pre>
 * qᵣ - qₗ = q̇ᵣ*h
 * Mᵣ(vᵣ - vₗ) = (B*uᵣ + Jᵣᵀ*λᵣ -c(qᵣ, vᵣ))h
 * </pre>
 * where
 * qᵣ: The generalized position on the right knot.
 * qₗ: The generalized position on the left knot.
 * vᵣ: The generalized velocity on the right knot.
 * vₗ: The generalized velocity on the left knot.
 * uᵣ: The actuator input on the right knot.
 * Mᵣ: The inertia matrix computed from qᵣ.
 * λᵣ: The constraint force (e.g., contact force, joint limit force, etc) on the
 * right knot.
 * c(qᵣ, vᵣ): The Coriolis, gravity and centripedal force on the right knot.
 * h: The duration between the left and right knot.
 */
DirectTranscriptionConstraint::DirectTranscriptionConstraint(
    const RigidBodyTree<double>& tree,
    std::shared_ptr<KinematicsCacheWithVHelper<AutoDiffXd>> kinematics_helper)
    : Constraint(tree.get_num_positions() + tree.get_num_velocities(),
                 1 + 2 * tree.get_num_positions() +
                     2 * tree.get_num_velocities() + tree.get_num_actuators(),
                 Eigen::VectorXd::Zero(tree.get_num_positions() +
                                       tree.get_num_velocities()),
                 Eigen::VectorXd::Zero(tree.get_num_positions() +
                                       tree.get_num_velocities())),
      tree_(&tree),
      num_positions_{tree.get_num_positions()},
      num_velocities_{tree.get_num_velocities()},
      num_actuators_{tree.get_num_actuators()},
      num_lambda_{0},
      kinematics_helper1_{kinematics_helper} {}

void DirectTranscriptionConstraint::AddGeneralizedConstraintForceEvaluator(
    std::unique_ptr<GeneralizedConstraintForceEvaluator> evaluator) {
  set_num_vars(num_vars() + evaluator->num_lambda());
  num_lambda_ += evaluator->num_lambda();
  generalized_constraint_force_evaluators_.push_back(std::move(evaluator));
}

void DirectTranscriptionConstraint::AddGeneralizedConstraintForceEvaluator(
    std::unique_ptr<GeneralizedConstraintForceEvaluator> evaluator,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& new_lambda_vars,
    solvers::VectorXDecisionVariable* x) {
  if (x->rows() != num_vars()) {
    throw std::runtime_error("x doesn't have the right size.");
  }
  if (new_lambda_vars.rows() != evaluator->num_lambda()) {
    throw std::runtime_error("new_lambda_vars doesn't have the right size.");
  }
  AddGeneralizedConstraintForceEvaluator(std::move(evaluator));
  // Append new_lambda_vars to the end of x, since lambda_r shows up in the end
  // of x, as defined in CompositeEvalInput(...)
  x->conservativeResize(num_vars());
  x->tail(new_lambda_vars.rows()) = new_lambda_vars;
}

void DirectTranscriptionConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), y_t);
  y = math::autoDiffToValueMatrix(y_t);
}

void DirectTranscriptionConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd& y) const {
  DRAKE_ASSERT(x.size() == num_vars());

  int x_count = 0;
  // A lambda expression to take num_element entreis from x, in a certain
  // order.
  auto x_segment = [x, &x_count](int num_element) {
    x_count += num_element;
    return x.segment(x_count - num_element, num_element);
  };

  const AutoDiffXd h = x(0);
  x_count++;
  const AutoDiffVecXd h_l = ;
  const AutoDiffVecXd h_r = ;
  const AutoDiffVecXd q_l = x_segment(num_positions_);
  const AutoDiffVecXd v_plus_l = x_segment(num_velocities_);
  const AutoDiffVecXd v_minus_l = x_segment(num_velocities_);
  const AutoDiffVecXd q_r = x_segment(num_positions_);
  const AutoDiffVecXd v_plus_r = x_segment(num_velocities_);  
  const AutoDiffVecXd v_minus_r = x_segment(num_velocities_);
  const AutoDiffVecXd u_r = x_segment(num_actuators_);
  const AutoDiffVecXd lambda_r = x_segment(num_lambda_);

  auto kinsol = kinematics_helper1_->UpdateKinematics(q_r, v_r);

  y.resize(num_constraints());

  // By using backward Euler integration, the constraint is
  // qᵣ - qₗ = q̇ᵣ*h
  // Mᵣ(vᵣ - vₗ) = (B*uᵣ + Jᵣᵀ*λᵣ -c(qᵣ, vᵣ))h
  const MatrixX<AutoDiffXd> map_v_to_qdot =
      RigidBodyTree<double>::GetVelocityToQDotMapping(kinsol);
  const AutoDiffVecXd qdot_plus_r = map_v_to_qdot * v_plus_r;
  const AutoDiffVecXd qdot_minus_r = map_v_to_qdot * v_minus_r;
  const AutoDiffVecXd qdot_plus_l = map_v_to_qdot * v_plus_l;
  const AutoDiffVecXd qdot_minus_l = map_v_to_qdot * v_minus_l;
  // TODO(hongkai.dai): Project qdot_r to the constraint manifold (for example,
  // if q contains unit quaternion, and we need to project this backward Euler
  // integration on the unit quaternion manifold.)
  y.head(num_positions_) = q_r - q_l - qdot_minus_r * (h_r - h_l);

  const auto M = tree_->massMatrix(kinsol);

  // Compute the Coriolis force, centripedal force, etc.
  const typename RigidBodyTree<AutoDiffXd>::BodyToWrenchMap
      no_external_wrenches;
  const auto c = tree_->dynamicsBiasTerm(kinsol, no_external_wrenches);

  // Compute Jᵀλ
  AutoDiffVecXd total_generalized_constraint_force(num_velocities_);
  total_generalized_constraint_force.setZero();
  int lambda_count = 0;
  for (const auto& evaluator : generalized_constraint_force_evaluators_) {
    AutoDiffVecXd q_v_lambda(num_positions_ + num_velocities_ +
                             evaluator->num_lambda());
    q_v_lambda << q_r, v_r,
        lambda_r.segment(lambda_count, evaluator->num_lambda());
    AutoDiffVecXd generalized_constraint_force(num_velocities_);
    evaluator->Eval(q_v_lambda, generalized_constraint_force);
    total_generalized_constraint_force += generalized_constraint_force;
    lambda_count += evaluator->num_lambda();
  }

  y.tail(num_velocities_) =
      M * (v_minus_r - v_plus_l) -
      (tree_->B * u_r + total_generalized_constraint_force - c) * (h_r - h_l);

  y.tail(num_velocities_) = 
      qdot_plus_l - qdot_minus_l + (1)*inverse(M)*total_generalized_constraint_force;
}
namespace {

// This class encodes the complementarity constraint on the joint limit
// constraint force λ, and the joint q.
// The constraints are
// (qᵤ - q) * λᵤ = 0
// (q - qₗ) * λₗ = 0
// where qᵤ is the joint upper bound, and qₗ is the joint lower bound.
// λᵤ / λₗ are the joint limit force from upper bound and lower bound
// respectively.
class JointLimitsComplementarityConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JointLimitsComplementarityConstraint)

  JointLimitsComplementarityConstraint(double joint_lower_bound,
                                       double joint_upper_bound)
      : solvers::Constraint(2, 3, Eigen::Vector2d::Zero(),
                            Eigen::Vector2d::Zero()),
        joint_lower_bound_(joint_lower_bound),
        joint_upper_bound_(joint_upper_bound) {}

  template <typename Scalar>
  Vector3<Scalar> CompositeEvalInput(const Scalar& q,
                                     const Scalar joint_lower_bound_force,
                                     const Scalar& joint_upper_bound_force) {
    return Vector3<Scalar>(q, joint_upper_bound_force, joint_lower_bound_force);
  }

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override {
    AutoDiffVecXd ty;
    Eval(math::initializeAutoDiff(x), ty);
    y = math::autoDiffToValueMatrix(ty);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override {
    y.resize(2);
    y(0) = (joint_upper_bound_ - x(0)) * x(1);
    y(1) = (x(0) - joint_lower_bound_) * x(2);
  }

 private:
  const double joint_lower_bound_;
  const double joint_upper_bound_;
};
}  // namespace

ElasiticContactImplicitDirectTranscription::ElasiticContactImplicitDirectTranscription(
    const RigidBodyTree<double>& tree, int num_time_samples,
    double minimum_timestep, double maximum_timestep)
    : MultipleShooting(tree.get_num_actuators(),
                       tree.get_num_positions() + tree.get_num_velocities(),
                       num_time_samples, minimum_timestep, maximum_timestep),
      tree_{&tree},
      num_positions_{tree.get_num_positions()},
      num_velocities_{tree.get_num_velocities()},
      position_constraint_lambda_vars_(NewContinuousVariables(
          tree.getNumPositionConstraints(), N(), "position_lambda")) {
  // For each knot, we will need to impose a transcription/collocation
  // constraint. Each of these constraints require us caching some
  // kinematics info.
  kinematics_cache_with_v_helpers_.resize(num_time_samples);
  for (int i = 0; i < num_time_samples; ++i) {
    kinematics_cache_with_v_helpers_[i] =
        std::make_shared<KinematicsCacheWithVHelper<AutoDiffXd>>(*tree_);
  }

  q_vars_.resize(num_positions_, N());
  v_vars_.resize(num_velocities_, N());
  for (int i = 0; i < N(); ++i) {
    q_vars_.col(i) = x_vars().segment(num_states() * i, num_positions_);
    v_vars_.col(i) =
        x_vars().segment(num_states() * i + num_positions_, num_velocities_);
  }

  direct_transcription_constraints_.reserve(N() - 1);
  for (int i = 0; i < N() - 1; ++i) {
    auto transcription_cnstr = std::make_shared<DirectTranscriptionConstraint>(
        *tree_, kinematics_cache_with_v_helpers_[i + 1]);
    // Add RigidBodyConstraint::PositionConstraint to the constraint force Jᵀλ
    // used in the dynamics for direct transcription.
    auto position_constraint_force_evaluator =
        std::make_unique<PositionConstraintForceEvaluator>(
            *tree_, kinematics_cache_with_v_helpers_[i + 1]);
    transcription_cnstr->AddGeneralizedConstraintForceEvaluator(
        std::move(position_constraint_force_evaluator));

    const solvers::VectorXDecisionVariable transcription_vars =
        transcription_cnstr->CompositeEvalInput(
            h_vars()(i), q_vars_.col(i), v_vars_.col(i), q_vars_.col(i + 1),
            v_vars_.col(i + 1),
            u_vars().segment((i + 1) * num_inputs(), num_inputs()),
            position_constraint_lambda_vars_.col(i + 1));
    direct_transcription_constraints_.emplace_back(transcription_cnstr,
                                                   transcription_vars);
  }
}

solvers::VectorDecisionVariable<2>
ElasiticContactImplicitDirectTranscription::AddJointLimitImplicitConstraint(
    int interval_index, int joint_position_index, int joint_velocity_index,
    double joint_lower_bound, double joint_upper_bound) {
  if (interval_index < 0 || interval_index > N() - 1) {
    throw std::runtime_error("interval_index is invalid.");
  }
  const int right_knot_index = interval_index + 1;
  const std::string lambda_name =
      "joint_" + std::to_string(joint_velocity_index) + "_limit_lambda[" +
      std::to_string(right_knot_index) + "]";
  // joint_limit_lambda[0] is lower limit force.
  // joint_limit_lambda[1] is upper limit force.
  const solvers::VectorDecisionVariable<2> joint_limit_lambda =
      NewContinuousVariables<2>(lambda_name);
  const symbolic::Variable lower_limit_force_lambda = joint_limit_lambda[0];
  const symbolic::Variable upper_limit_force_lambda = joint_limit_lambda[1];
  // λᵤ ≥ 0, λₗ ≥ 0
  AddBoundingBoxConstraint(0, std::numeric_limits<double>::infinity(),
                           joint_limit_lambda);
  // qₗ ≤ q ≤ qᵤ
  AddBoundingBoxConstraint(joint_lower_bound, joint_upper_bound,
                           q_vars_(joint_position_index, right_knot_index));
  // Adds the joint limit force to the constraint force.
  auto joint_limit_force_evaluator =
      std::make_unique<JointLimitConstraintForceEvaluator>(
          *tree_, joint_velocity_index);
  solvers::VectorDecisionVariable<2> joint_limit_force_evaluator_lambda;
  joint_limit_force_evaluator_lambda(
      JointLimitConstraintForceEvaluator::LowerLimitForceIndexInLambda()) =
      lower_limit_force_lambda;
  joint_limit_force_evaluator_lambda(
      JointLimitConstraintForceEvaluator::UpperLimitForceIndexInLambda()) =
      upper_limit_force_lambda;

  AddGeneralizedConstraintForceEvaluatorToTranscription(
      interval_index, std::move(joint_limit_force_evaluator),
      joint_limit_force_evaluator_lambda);

  // Add the complementarity constraint
  // (qᵤ - q) * λᵤ = 0
  // (q - qₗ) * λₗ = 0
  auto joint_complementary_constraint =
      std::make_shared<JointLimitsComplementarityConstraint>(joint_lower_bound,
                                                             joint_upper_bound);
  const auto joint_complementary_vars =
      joint_complementary_constraint->CompositeEvalInput(
          q_vars_(joint_position_index, right_knot_index),
          lower_limit_force_lambda, upper_limit_force_lambda);
  AddConstraint(joint_complementary_constraint, joint_complementary_vars);
  return joint_limit_lambda;
}

solvers::VectorDecisionVariable<8>
ElasiticContactImplicitDirectTranscription::AddContactImplicitConstraint(
    int interval_index)
{
  if (interval_index < 0 || interval_index > N() - 1) {
    throw std::runtime_error("interval_index is invalid.");
  }

  const std::string lambda_name =
      "contact_implicit_constraint_[" + std::to_string(interval_index) + "]";
  const solvers::VectorDecisionVariable<8> contact_implicit_lambda =
      NewContinuousVariables<8>(lambda_name);



  return contact_implicit_lambda;
}

void ElasiticContactImplicitDirectTranscription::Compile() {
  for (int i = 0; i < N() - 1; ++i) {
    AddConstraint(direct_transcription_constraints_[i].constraint(),
                  direct_transcription_constraints_[i].variables());
  }
}

void ElasiticContactImplicitDirectTranscription::
    AddGeneralizedConstraintForceEvaluatorToTranscription(
        int interval_index,
        std::unique_ptr<GeneralizedConstraintForceEvaluator> evaluator,
        const Eigen::Ref<const solvers::VectorXDecisionVariable>&
            evaluator_lambda) {
  DRAKE_ASSERT(evaluator->num_lambda() == evaluator_lambda.rows());
  solvers::VectorXDecisionVariable vars =
      direct_transcription_constraints_[interval_index].variables();
  auto direct_transcription_constraint =
      direct_transcription_constraints_[interval_index].constraint();
  direct_transcription_constraint->AddGeneralizedConstraintForceEvaluator(
      std::move(evaluator), evaluator_lambda, &vars);
  // Now update the Binding in direct_transcription_constraints_
  direct_transcription_constraints_[interval_index] =
      solvers::Binding<DirectTranscriptionConstraint>(
          direct_transcription_constraint, vars);
}

void ElasiticContactImplicitDirectTranscription::DoAddRunningCost(
    const symbolic::Expression& g) {
  // Add the running cost ∫ g(t, x, u)
  // We discretize this continuous integration as
  // sum_{i = 0, ..., N - 2} h_i * g_{i+1}
  for (int i = 0; i < N() - 2; ++i) {
    AddCost(SubstitutePlaceholderVariables(g * h_vars()(i), i + 1));
}

PiecewisePolynomialTrajectory
ElasiticContactImplicitDirectTranscription::ReconstructStateTrajectory() const {
  Eigen::VectorXd times = GetSampleTimes();
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> states(N());

  for (int i = 0; i < N(); ++i) {
    times_vec[i] = times(i);
    states[i] = GetSolution(state(i));
  }
  return PiecewisePolynomialTrajectory(
      PiecewisePolynomial<double>::FirstOrderHold(times_vec, states));
}

PiecewisePolynomialTrajectory
ElasiticContactImplicitDirectTranscription::ReconstructInputTrajectory() const {
  Eigen::VectorXd times = GetSampleTimes();
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> inputs(N());

  for (int i = 0; i < N(); ++i) {
    times_vec[i] = times(i);
    inputs[i] = GetSolution(input(i));
  }
  return PiecewisePolynomialTrajectory(
      PiecewisePolynomial<double>::ZeroOrderHold(times_vec, inputs));
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
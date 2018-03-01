#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/kinematics_cache_helper.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/trajectory_optimization/generalized_constraint_force_evaluator.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"
#include "drake/systems/trajectory_optimization/rigid_body_tree_multiple_shooting_internal.h"
#include "drake/systems/trajectory_optimization/contact_implicit_constraint.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {


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
 * Note that the robot might have many generalized constraint forces, such as
 * loop joint forces, ground contact forces, joint limit forces, etc. These
 * generalized constraint forces are additive, namely if the loop joint forces
 * are Jₗₒₒₚᵀ*λₗₒₒₚ, and the joint constraint forces are Jⱼₒᵢₙₜᵀ*λⱼₒᵢₙₜ, then
 * the total generalized constraint forces from both the loop joint are the
 * joint limits are ther sum Jₗₒₒₚᵀ*λₗₒₒₚ + Jⱼₒᵢₙₜᵀ*λⱼₒᵢₙₜ
 * So we will store a vector of GeneralizedConstraintForceEvaluator in this
 * class, each representing one term of generalized constraint force, and we
 * will evaluate each one of them to obtain the summed constraint forces.
 */
class CustomDirectTranscriptionConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CustomDirectTranscriptionConstraint)

  /** Constructor
   * @param tree The RigidBodyTree whose trajectory will be optimized.
   * @param num_lambda The number of lambda on the right knot point.
   * @param kinematics_helper The kinematics helper that stores the kinematics
   * information for the position and velocity on the right knot.
   */
  CustomDirectTranscriptionConstraint(
      const RigidBodyTree<double>& tree,
      std::shared_ptr<plants::KinematicsCacheWithVHelper<AutoDiffXd>>
          kinematics_helper);

  ~CustomDirectTranscriptionConstraint() override = default;

  /**
   * Adds a GeneralizedConstraintForceEvaluator.
   * Note that by adding a new generalized constraint force evaluator, it also
   * increases CustomDirectTranscriptionConstraint::num_vars() by
   * evaluator.num_lambda(). Namely the last evaluator.num_lambda() variables
   * bounded with this CustomDirectTranscriptionConstraint, is going to be used to
   * compute the generalized constraint force in this @p evaluator.
   * @param evaluator This will evaluate a generalized constraint force. If the
   * user has evaluator1 that computes the loop joint force, and evaluator2 that
   * computes the contact force from the foot toe, then the user can call this
   * function for twice
   * AddGeneralizedConstraintForceEvaluator(evaluator1);
   * AddGeneralizedConstraintForceEvaluator(evaluator2);
   * The user doesn't have to create one evaluator, that computes the summation
   * of all generalized constraint forces.
   */
  void AddGeneralizedConstraintForceEvaluator(
      std::unique_ptr<GeneralizedConstraintForceEvaluator> evaluator);

  void AddGeneralizedConstraintForceEvaluator(
      std::unique_ptr<GeneralizedConstraintForceEvaluator> evaluator,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& new_lambda_vars,
      solvers::VectorXDecisionVariable* x);

  /**
   * Composite the input variable x to the Eval function, from the state,
   * control and contact force at left or the right knot. This function helps
   * the users so that they do not need to memorize the order of variables in
   * `x`, used in the Eval function.
   */
  template <typename Scalar, typename DerivedQL, typename DerivedVL,
            typename DerivedQR, typename DerivedVR, typename DerivedUR,
            typename DerivedLambdaR>
  typename std::enable_if<is_eigen_vector_of<DerivedQL, Scalar>::value &&
                              is_eigen_vector_of<DerivedVL, Scalar>::value &&
                              is_eigen_vector_of<DerivedQR, Scalar>::value &&
                              is_eigen_vector_of<DerivedVR, Scalar>::value &&
                              is_eigen_vector_of<DerivedUR, Scalar>::value &&
                              is_eigen_vector_of<DerivedLambdaR, Scalar>::value,
                          Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>::type
  CompositeEvalInput(const Scalar& h, const Eigen::MatrixBase<DerivedQL>& q_l,
                     const Eigen::MatrixBase<DerivedVL>& v_l,
                     const Eigen::MatrixBase<DerivedQR>& q_r,
                     const Eigen::MatrixBase<DerivedVR>& v_r,
                     const Eigen::MatrixBase<DerivedUR>& u_r,
                     const Eigen::MatrixBase<DerivedLambdaR>& lambda_r) const {
    DRAKE_ASSERT(q_l.rows() == num_positions_);
    DRAKE_ASSERT(v_l.rows() == num_velocities_);
    DRAKE_ASSERT(q_r.rows() == num_positions_);
    DRAKE_ASSERT(v_r.rows() == num_velocities_);
    DRAKE_ASSERT(u_r.rows() == num_actuators_);
    DRAKE_ASSERT(lambda_r.rows() == num_lambda_);
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> x(num_vars(), 1);
    x << h, q_l, v_l, q_r, v_r, u_r, lambda_r;
    return x;
  }

  template <typename Scalar, typename DerivedQL, typename DerivedVL,
            typename DerivedQR, typename DerivedVR, typename DerivedUR>
  typename std::enable_if<is_eigen_vector_of<DerivedQL, Scalar>::value &&
                              is_eigen_vector_of<DerivedVL, Scalar>::value &&
                              is_eigen_vector_of<DerivedQR, Scalar>::value &&
                              is_eigen_vector_of<DerivedVR, Scalar>::value &&
                              is_eigen_vector_of<DerivedUR, Scalar>::value,
                          Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>::type
  CompositeEvalInput(const Scalar& h, const Eigen::MatrixBase<DerivedQL>& q_l,
                     const Eigen::MatrixBase<DerivedVL>& v_l,
                     const Eigen::MatrixBase<DerivedQR>& q_r,
                     const Eigen::MatrixBase<DerivedVR>& v_r,
                     const Eigen::MatrixBase<DerivedUR>& u_r) const {
    DRAKE_ASSERT(q_l.rows() == num_positions_);
    DRAKE_ASSERT(v_l.rows() == num_velocities_);
    DRAKE_ASSERT(q_r.rows() == num_positions_);
    DRAKE_ASSERT(v_r.rows() == num_velocities_);
    DRAKE_ASSERT(u_r.rows() == num_actuators_);
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> x(num_vars(), 1);
    x << h, q_l, v_l, q_r, v_r, u_r;
    return x;
  }

  template <typename Scalar, typename DerivedQL, typename DerivedVL,
            typename DerivedQR, typename DerivedVR, typename DerivedUR>
  typename std::enable_if<is_eigen_vector_of<DerivedQL, Scalar>::value &&
                              is_eigen_vector_of<DerivedVL, Scalar>::value &&
                              is_eigen_vector_of<DerivedQR, Scalar>::value &&
                              is_eigen_vector_of<DerivedVR, Scalar>::value &&
                              is_eigen_vector_of<DerivedUR, Scalar>::value,
                          Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>::type
  CompositeEvalInput(const Scalar& h, const Eigen::MatrixBase<DerivedQL>& q_l,
                     const Eigen::MatrixBase<DerivedVL>& v_plus_l,
                     const Eigen::MatrixBase<DerivedVL>& v_minus_l,
                     const Eigen::MatrixBase<DerivedQR>& q_r,
                     const Eigen::MatrixBase<DerivedVR>& v_plus_r,
                     const Eigen::MatrixBase<DerivedVR>& v_minus_r,
                     const Eigen::MatrixBase<DerivedUR>& u_r) const {
    DRAKE_ASSERT(q_l.rows() == num_positions_);
    DRAKE_ASSERT(v_plus_l.rows() == num_velocities_/2);
    DRAKE_ASSERT(v_minus_l.rows() == num_velocities_/2);
    DRAKE_ASSERT(q_r.rows() == num_positions_);
    DRAKE_ASSERT(v_plus_r.rows() == num_velocities_/2);
    DRAKE_ASSERT(v_minus_r.rows() == num_velocities_/2);
    DRAKE_ASSERT(u_r.rows() == num_actuators_);
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> x(num_vars(), 1);
    x << h, q_l, v_plus_l, v_minus_l, q_r, v_plus_r, v_minus_r, u_r;
    return x;
  }

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

 private:
  const RigidBodyTree<double>* tree_;
  const int num_positions_;
  const int num_velocities_;
  const int num_actuators_;
  int num_lambda_;
  // Stores the kinematics cache at the right knot point.
  mutable std::shared_ptr<plants::KinematicsCacheWithVHelper<AutoDiffXd>>
      kinematics_helper1_;
  // Stores the GeneralizedConstraintForceEvaluator
  std::vector<std::unique_ptr<GeneralizedConstraintForceEvaluator>>
      generalized_constraint_force_evaluators_;
};


/**
 * Implements the trajectory optimization for a RigidBodyTree.
 * Trajectory optimization for RigidBodyTree is special, because the dynamics
 * of the tree has some special structures.
 * 1. Since RigidBodyTree has a second order dynamics, its dynamics can be
 * separated as the time derivative on the generalized position, and the time
 * derivative on the generalized velocities.
 * 2. Its generalized acceleration can be affected by the external force, under
 * the term Jᵀλ. We can optimize over λ as decision variables.
 * 3. The kinematics cache can be reused in each knot of the trajectory, so we
 * will store the kinematics cache for each knot.
 *
 * By default, the generalized constraint force Jᵀλ only includes those from
 * RigidBodyTree::PositionConstraint().
 *
 * @note The user MUST call this Compile function before solving the
 * optimization program, and after all the generalized constraint force Jᵀλ has
 * been added to the program.
 */
class ElasticContactImplicitDirectTranscription : public MultipleShooting {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ElasticContactImplicitDirectTranscription)

  /**
   * Constructor.
   * @param tree The RigidBodyTree whose trajectory will be optimized.
   * @param num_time_samples The total number of knots in the trajectory.
   * @param minimum_timestep The minimum of the time step.
   * @param maximum_timestep The maximum of the time step.
   */
  ElasticContactImplicitDirectTranscription(const RigidBodyTree<double>& tree,
                                const RigidBodyTree<double>& empty_tree,
                                int num_time_samples, double minimum_timestep,
                                double maximum_timestep, int num_contact_lambda,
                                double compl_tol, double elasticity);

  PiecewisePolynomialTrajectory ReconstructInputTrajectory() const override;

  PiecewisePolynomialTrajectory ReconstructStateTrajectory() const override;

  const solvers::MatrixXDecisionVariable& GeneralizedPositions() const {
    return q_vars_;
  }

  const solvers::MatrixXDecisionVariable& GeneralizedVelocities() const {
    return v_vars_;
  }

  const solvers::MatrixXDecisionVariable& ContactConstraintForces() const {
    return lambda_vars_;
  }

  ~ElasticContactImplicitDirectTranscription() override {}

  const RigidBodyTree<double>* tree() const { return tree_; }

  /** Getter for the kinematics cache helper. */
  // const std::shared_ptr<plants::KinematicsCacheWithVHelper<AutoDiffXd>>
  // kinematics_cache_with_v_helpers(int index) const {
  //   return kinematics_cache_with_v_helpers_[index];
  // }
  // const std::shared_ptr<plants::KinematicsCacheHelper<AutoDiffXd>>
  // kinematics_cache_helpers(int index) const {
  //   return kinematics_cache_helpers_[index];
  // }

  /**
   * Adds the direct transcription constraint to the optimization program.
   * The user MUST call this Compile function before solving the optimization
   * program, and after all the generalized constraint force Jᵀλ has been
   * added to the program.
   */
  void Compile();

 protected:
  int num_positions() const { return num_positions_; }

  int num_velocities() const { return num_velocities_; }

  void AddGeneralizedConstraintForceEvaluatorToTranscription(
      int interval_index,
      std::unique_ptr<GeneralizedConstraintForceEvaluator> evaluator,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>&
          evaluator_lambda);

 private:
  void DoAddRunningCost(const symbolic::Expression& e) override;

  // Store system-relevant data for e.g. computing the derivatives during
  // trajectory reconstruction.
  const RigidBodyTree<double>* tree_{nullptr};
  const RigidBodyTree<double>* empty_tree_{nullptr};
  const int num_positions_;
  const int num_velocities_;
  const int num_lambda_;
  const double compl_tol_;
  const double elasticity_;
  // const int num_contacts_ = 8; // Aditya: in the future this shouldn't be hard coded but i'm using a box...
  // direct_transcription_constraints_[i] stores the
  // CustomDirectTranscriptionConstraint
  // between knot i and i+1, together with the bounded variables. Notice that
  // when the user adds additional constraint force knot i,
  // direct_transcription_constraints_[i-1] needs to add the constraint force
  // evaluator, and the bound variables should be updated to include the new
  // constraint force λ.
  std::vector<solvers::Binding<CustomDirectTranscriptionConstraint>>
      direct_transcription_constraints_;
  std::vector<solvers::Binding<TimestepIntegrationConstraint>>
      timestep_integration_constraints_;
  std::vector<solvers::Binding<ContactImplicitConstraint>>
      contact_implicit_constraints_;

  std::vector<std::shared_ptr<plants::KinematicsCacheWithVHelper<AutoDiffXd>>>
      dtc_kinematics_cache_with_v_helpers_;
  std::vector<std::shared_ptr<plants::KinematicsCacheHelper<AutoDiffXd>>>
      dtc_kinematics_cache_helpers_;

  std::vector<std::shared_ptr<plants::KinematicsCacheWithVHelper<AutoDiffXd>>>
      tic_kinematics_cache_with_v_helpers_;

  std::vector<std::shared_ptr<plants::KinematicsCacheWithVHelper<AutoDiffXd>>>
      cic_kinematics_cache_with_v_helpers_;

  solvers::MatrixXDecisionVariable q_vars_;
  solvers::MatrixXDecisionVariable v_vars_;
  solvers::MatrixXDecisionVariable lambda_vars_;
  solvers::MatrixXDecisionVariable position_constraint_lambda_vars_;
};
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake


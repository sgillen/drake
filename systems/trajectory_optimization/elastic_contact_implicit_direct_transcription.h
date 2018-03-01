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
  // DirectTranscriptionConstraint
  // between knot i and i+1, together with the bounded variables. Notice that
  // when the user adds additional constraint force knot i,
  // direct_transcription_constraints_[i-1] needs to add the constraint force
  // evaluator, and the bound variables should be updated to include the new
  // constraint force λ.
  std::vector<solvers::Binding<DirectTranscriptionConstraint>>
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


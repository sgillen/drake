#pragma once

#include <drake/solvers/mathematical_program.h>

#include "drake/common/nice_type_name.h"
#include "drake/manipulation/estimators/dev/dart_util.h"

namespace drake {
namespace manipulation {

/**
 * A scene that DART is to be used for estimation.
 * Contains a scene model, the instances within tscene, and dictates
 * what joints are being optimized over vs. which ones are direct pass-through.
 */
// TODO(eric.cousineau): Consider merging with WorldSimBuilder.
class DartScene {
 public:
  DartScene(TreePtr tree,
            const InstanceIdMap& instance_id_map);

  int GetInstanceId(const string& name) const {
    return instance_id_map_.at(name);
  }
  const RigidBodyTreed& tree() const {
    return *tree_;
  }

  const vector<string>& position_names() const {
    return position_names_;
  }

  const vector<string>& velocity_names() const {
    return velocity_names_;
  }

  KinematicsSlice CreateKinematicsSlice(
      const vector<string>& sub_positions,
      const vector<string>& sub_velocities) const;

 private:
  TreePtr tree_;
  InstanceIdMap instance_id_map_;
  ReverseIdMap instance_name_map_;
  vector<string> position_names_;
  vector<string> velocity_names_;
};

class DartObjective;
typedef vector<unique_ptr<DartObjective>> DartObjectiveList;

/**
 * Contains information regarding formulation of tracking problem, such as the
 * objectives, the variables they create, etc. Will NOT store anything about
 * the state of the solution (e.g., initial condition (aside from the scene),
 * etc.).
 */
// TODO(eric.cousineau): Consider minimal formulation such that non-estimated
// states (inputs) are incoprorated as strict equality constraints. Should do
// some basic performance testing / qualitative testing.
class DartFormulation {
 public:
  struct Param {
    vector<string> estimated_positions;
    vector<string> estimated_velocities;
  };

  DartFormulation(unique_ptr<DartScene> scene, const Param& param);

  /**
   * Create and add an objective, setting this as the formulation.
   */
  template <typename Obj, typename ... Args>
  Obj* AddObjective(Args&&... args) {
    Obj* objective = new Obj(this, std::forward<Args>(args)...);
    AddObjective(CreateUnique(objective));
  }

  /**
   * Add and initialize an objective for this formulation.
   */
  void AddObjective(unique_ptr<DartObjective> objective);

  // To be called by DartEstimator. Get slices for each objective.
  void PostInit();

  const KinematicsSlice& kinematics_est_slice() const {
    return kinematics_est_slice_;
  }
  const KinematicsSlice& kinematics_nonest_slice() const {
    return kinematics_nonest_slice_;
  }
  const KinematicsSlice& kinematics_est_var_slice() const {
    return kinematics_est_var_slice_;
  }
  const KinematicsVars& kinematics_est_vars() const {
    return kinematics_est_vars_;
  }

  const DartObjectiveList& objectives() const { return objectives_; }
  const VectorSlice& objective_var_slice(int i) const {
    return objective_var_slices_.at(i);
  }

  /**
   * This is a VERY relaxed interface for objectives to access.
   * They should NOT attempt to call solve.
   * Additionally, no attempt should be made to obtain the prior solution state.
   * Rather, this will be passed in via DartObjective::Update().
   */
  MathematicalProgram& prog() { return prog_; }
  const RigidBodyTreed& tree() const { return scene_->tree(); }
private:
  Param param_;
  unique_ptr<DartScene> scene_;
  MathematicalProgram prog_;
  KinematicsVars kinematics_est_vars_;
  KinematicsSlice kinematics_est_slice_;
  KinematicsSlice kinematics_nonest_slice_;

  DartObjectiveList objectives_;
  // Track which values are use for the optimization.
  KinematicsSlice kinematics_est_var_slice_;
  vector<VectorSlice> objective_var_slices_;
  bool initialized_{};
};

MatrixXd CreateDefaultCovarianceMatrix(
    int num_vars,
    double initial_uncorrelated_variance = 1e-10);

/**
 * An QP EKF objective may have induce both costs and constraints.
 * The objective will NOT produce decision variables for the independent
 * kinematic variables (position, velocity).
 * Rather, it will produce any additional slack variables it needs, and rely
 * upon prior values provided via `UpdateFormulation()` for state (aside from
 * basic caching).
 */
// TODO(eric.cousineau): Look into using a System-compatible but not System-
// required storage mode for state. (do not worry about inputs).
// Refer to Alejandro's MultiBodyTree formulation, and usage of a System-like
// context and cache.
class DartObjective {
 public:
  DartObjective(DartFormulation* formulation)
      : formulation_(formulation) {}
  virtual ~DartObjective() {}

  /**
   * Initialize the objective, adding costs and/or constraints.
   * @param cache The kinematics for the initial configuration of the scene.
   */
  virtual void Init(const KinematicsCached& cache) = 0;

  /// Get optimization variables for given objective.
  virtual const OptVars& GetVars() const = 0;

  /// Get initial values for above values.
  virtual const VectorXd& GetInitialValues() const = 0;

  virtual MatrixXd GetInitialCovariance() const {
    // TODO(eric.cousineau): Figure out theoretically succinct initialization.
    return CreateDefaultCovarianceMatrix(GetVars().size());
  }

  virtual bool RequiresObservation() const {
    return true;
  }

  /**
   * Update problem formulation given priors, new inputs, new observations for
   * joint states (provided from `cache`
   * @param t
   * @param cache Cache from kinematics computed with estimated priors
   * (`x_q{k-1}`) and non-estimated inputs (`u_q{k}`).
   * @see DartEstimator::ObserveAndInputKinematicsState for more details.
   * @param obj_prior
   */
  virtual void UpdateFormulation(
      double t,
      const KinematicsCached& cache,
      const VectorXd& obj_prior) = 0;

  string name() const {
    return NiceTypeName::Get(*this);
  }

  /**
   * Each individual subclass must implement its own Observe() functionality
   * to take in (a) time and (b) measurement data (joint states, image, etc.).
   */
  double latest_observation_time() const {
    return latest_observation_time_;
  }

 protected:
  void set_latest_observation_time(double time) {
    latest_observation_time_ = time;
  }
  DartFormulation& formulation() const { return *formulation_; }
  const RigidBodyTreed& tree() const { return formulation_->tree(); }
  // Convenience accessors.
  const OptVars& q_est_vars() const {
    return formulation().kinematics_est_vars().q();
  }
  MathematicalProgram& prog() const { return formulation_->prog(); }
 private:
  DartFormulation* formulation_{};
  double latest_observation_time_{};
};

// TODO(eric.cousineau): Consider formulation necessary for multi-rate
// esimators, such as in Cifuentes et al. - note that this is a particle
// filter, and may not fit into this framework at all.
// TODO(eric.cousineau): Consider formulation for adding / removing bodies,
// where the number of variables needs to change on the fly.
class DartEstimator {
 public:
  struct Param {
    // This MUST be initialized.
    KinematicsState initial_state;
    double max_observed_time_diff {1e-4};
  };

  DartEstimator(unique_ptr<DartFormulation> formulation, const Param& param);

  /**
   * Take frame k's non-estimated states as inputs (u_q{k}), and take frame k's
   * measurements as observations (v_q{k}).
   * Since kinematic states are first-class citizens in this esimtator, this
   * will be done by merging the cache using both the kinematic state
   * prior (x_q{k-1}) and the non-estimated states (u_q{k}), and then providing
   * the observations of the estimated state (v_q{k}) for consumption by other
   * objectives by means of the kinematics cache (`getQ` and `getV`).
   */
  void ObserveAndInputKinematicsState(
      double t, const KinematicsState& state_meas);

  const KinematicsState& Update(double t);

  const KinematicsState& state_prior_with_input(double t) const {
    CheckObservationTime(t, latest_state_observation_time_, "state_prior");
    return state_prior_with_input_;
  }

  const KinematicsState& state_meas(double t) const {
    CheckObservationTime(t, latest_state_observation_time_, "state_meas");
    return state_meas_;
  }

  const KinematicsState& initial_state() const {
    return param_.initial_state;
  }

 protected:
  // Initialize all objectives, and ensure that the optimization problem is
  // ready to be performed.
  void Compile();

  // Ensure an observation time is up-to-date with the present time.
  void CheckObservationTime(double t_now, double t_obs,
                            const string& name) const;

  void UpdateKinematicsWithPriorAndInput();

  // Convenience accessors.
  const RigidBodyTreed& tree() const { return formulation_->tree(); }
  MathematicalProgram& prog() { return formulation_->prog(); }
  const DartObjectiveList& objectives() { return formulation_->objectives(); }
 private:
  Param param_;
  unique_ptr<DartFormulation> formulation_;

  // This is the measured state, q_meas{k}, for both estimated and
  // non-estimated states.
  double latest_state_observation_time_{};
  KinematicsState state_meas_;

  // `input` in this case means non-estimated state measurements, such that we
  // combine q_est{k-1} (prior) with input q_nonest_meas{k} (input).
  // TODO(eric.cousineau): Don't store combined state, it's confusing. Rather,
  // store state_est_prior_ and state_nonest_input_ as slices.
  KinematicsState state_prior_with_input_;
  KinematicsCached cache_prior_with_input_;

  MatrixXd covariance_;
  VectorXd opt_val_prior_;
};

}  // manipulation
}  // drake

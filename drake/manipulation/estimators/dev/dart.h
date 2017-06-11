#include <drake/solver/mathematical_program.h>

#include "drake/manipulation/estimation/dev/dart_util.h"

namespace drake {
namespace manipulation {

using namespace drake::solvers;
using namespace std;

// Contains values
struct KinematicsState {
  // Full kinematic state, including non-decision variables.
  VectorXd q;
  VectorXd v;
};

struct KinematicsSlice {
  VectorSlice q;
  VectorSlice v;
};

struct KinematicsVars {
  OptVars q;
  OptVars v;
};

void DemandAllVariablesHaveUniqueNames(const OptVars& vars) {
  set<string> names;
  for (auto&& var : vars) {
    // Attempt to insert.
    auto result = names.insert(var.get_name());
    bool is_unique = result.second;
    DRAKE_DEMAND(is_unique);
  }
}

/**
 * A scene that DART is to be used for estimation.
 * Contains a scene model, the instances within the scene, and dictates
 * what joints are being optimized over vs. which ones are direct pass-through.
 */
// TODO(eric.cousineau): Consider merging with WorldSimBuilder.
class DartScene {
 public:
  DartScene(TreePtr tree,
            const InstanceIdMap& instance_id_map)
      : tree_(tree),
        instance_id_map_(instance_id_map) {}
  int GetInstanceId(const string& name) const {
    return instance_id_map_.at(name);
  }
  TreePtr tree() const {
    return tree_;
  }
  const InstanceIdMap& instance_id_map() const {
    return instance_id_map_;
  }
 private:
  TreePtr tree_;
  InstanceIdMap instance_id_map_;
};

/**
 * Contains information regarding formulation of tracking problem.
 */
class DartFormulation {
 public:
  DartFormulation(unique_ptr<DartScene> scene)
      : scene_(scene) {
    // Add
  }

  /// @sec Access from DartEstimator

  /**
   * @brief Set up the specific formulation after objectives are added to the
   * MathematicalProgram.
   */
  virtual void SetupWithObjectives() = 0;

  /// @sec Access from DartObjective

  /**
   * This is a VERY relaxed interface for objectives to access.
   * They should NOT attempt to call solve.
   */
  MathematicalProgram& prog() {
    return prog_;
  }
  TreePtr tree() const {
    return scene_.tree();
  }
private:
  unique_ptr<DartScene> scene_;
  MathematicalProgram prog_;
  KinematicsState qv_meas_;
  KinematicsState qv_prior_;
  KinematicsCached cache_prior_;
};

/**
 * EKF formulation pursuant to Greggory Izatt's formulation in his ICRA 2016
 * paper.
 */
class DartFormulationEKF : public DartFormulation {
 public:
  virtual void SetupWithObjectives() {
    // Initialize covariance per objective.

  }
 private:
};

/**
 * An objective may have induce both costs and constraints.
 * The objective will NOT produce decision variables for the independent
 * kinematic variables (position, velocity).
 * Rather, it will produce any additional slack variables it needs, and
 * store its own state internally.
 */
// TODO(eric.cousineau): Look into using a System-compatible but not System-
// required storage mode for state. (do not worry about inputs).
// Refer to Alejandro's MultiBodyTree formulation, and usage of a System-like
// context and cache.
class DartObjective {
 public:
  typedef const DartObjective* Hash;

  // TODO(eric.cousineau): See if there is a way to remove this.
  static const double kInitialUncorrelatedVariance = 1e-10;

  DartObjective(DartFormulation* formulation)
      : formulation_(formulation) {}

  /**
   * Initialize the objective, adding costs and/or constraints.
   */
  virtual void Init() = 0;

  virtual const OptVar& opt_vars() = 0;

  virtual void Update(
      const KinematicsCache* cache_prior,
      const VectorXd& ) = 0;

  // Each individual subclass must implement its own Observe() functionality
  // to take in (a) time and (b) measurement data (joint states, image, etc.).

  double get_latest_observation_time() const {
    return latest_measurement_time_;
  }

  // NOTE: 

  virtual MatrixXd GetInitialCovariance() const {
    // TODO(eric.cousineau): See if there is a better way to do this, per
    // Greg's comments.
    const int num_vars = opt_vars().size();
    return kInitialUncorrelatedVariance *
        MatrixXd::Identity(num_vars, num_vars);
  }

  // HACK?
  Hash hash() const { return this; }

 protected:
  void set_latest_observation_time(double time) {
    latest_observation_time_ = time;
  }
 private:
  DartFormulation* formulation_{};
  double latest_observation_time_{};
};

typedef vector<shared_ptr<DartObjective>> DartObjectiveList;


class DartJointObjective : public DartObjective {
 public:
  void Observe(const KinematicsState& full_state) {
  }
 private:
};


// TODO(eric.cousinaeu): Consider formulation necessary for multi-rate
// esimators, such as in Cifuentes et al. - note that this is a particle
// filter, and may not fit into this framework at all.
class DartEstimator {
 public:
  Dart(
    TreePtr scene_tree,
    Indices estimated_indices) {
  }

  void Update() {
    for (auto objective : objectives_) {
      objective->Update(formulation_);
    }
    formulation_.Solve();
  }

 private:
  DartFormulation formulation_;
  DartObjectiveList objectives_;
};

}  // manipulation
}  // drake

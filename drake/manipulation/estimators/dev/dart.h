#include <drake/solvers/mathematical_program.h>

#include "drake/common/nice_type_name.h"
#include "drake/manipulation/estimators/dev/dart_util.h"

namespace drake {
namespace manipulation {

using namespace drake::solvers;
using namespace std;

// Contains values
// Full kinematic state, including non-decision variables.
class KinematicsState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(KinematicsState);

  KinematicsState(int nq, int nv) {
    q_.resize(nq);
    q_.setZero();
    v_.resize(nv);
    v_.setZero();
  }
  KinematicsState(const RigidBodyTreed* tree)
    : KinematicsState(tree->get_num_positions(), tree->get_num_velocities()) {}

  Eigen::Ref<VectorXd> q() { return q_; }
  const VectorXd& q() const { return q_; }
  Eigen::Ref<VectorXd> v() { return v_; }
  const VectorXd& v() const { return v_; }

  VectorXd x() const {
    VectorXd out(q_.rows() + v_.rows());
    out << q_, v_;
    return out;
  }
 private:
  VectorXd q_;
  VectorXd v_;
};

/**
 * A partial view into independent kinematic state variables (position and
 * velocity) for a mechanical system.
 */
class KinematicsSlice {
 public:
  KinematicsSlice(const RigidBodyTreed& tree, const Indices& q_indices,
                  const Indices& v_indices)
      : q_(q_indices, tree.get_num_positions()),
        v_(v_indices, tree.get_num_velocities()) {}

  const VectorSlice& q() const { return q_; }
  const VectorSlice& v() const { return v_; }

  // Can be State, or something else (such as joint names, decision variables, etc.)
  template <typename KinematicsValues>
  void ReadFromSuperset(const KinematicsValues& super, KinematicsValues& sub) {
    q_.ReadFromSuperset(super.q(), sub.q());
    v_.ReadFromSuperset(super.v(), sub.v());
  }

  template <typename KinematicsValues>
  void WriteToSuperset(const KinematicsValues& sub, KinematicsValues& super) {
    q_.WriteToSuperset(sub.q(), super.q());
    v_.WriteToSuperset(sub.v(), super.v());
  }

 private:
  VectorSlice q_;
  VectorSlice v_;
};

class KinematicsVars {
 public:
  KinematicsVars() {}
  KinematicsVars(const OptVars& q, const OptVars& v)
      : q_(q), v_(v) {}
  // Permit this to be mutable.
  OptVars& q() { return q_; }
  const OptVars& q() const { return q_; }
  OptVars& v() { return v_; }
  const OptVars& v() const { return v_; }
 private:
  OptVars q_;
  OptVars v_;
};

void DemandAllVariablesHaveUniqueNames(const OptVars& vars) {
  set<string> names;
  for (auto&& var : MakeIterableMatrix(vars)) {
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
            const KinematicsState& initial_state,
            const InstanceIdMap& instance_id_map)
      : tree_(tree),
        initial_state_(initial_state),
        instance_id_map_(instance_id_map) {
    instance_name_map_ = ReverseMap(instance_id_map);
    GetHierarchicalKinematicNameList(this->tree(), instance_name_map_,
                                     &position_names_, &velocity_names_);
  }

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

  const KinematicsState& initial_state() const {
    return initial_state_;
  }

  KinematicsSlice CreateKinematicsSlice(
      const vector<string>& sub_positions,
      const vector<string>& sub_velocities) const {
    // Get slices.
    vector<int> q_subindices;
    GetCommonIndices(sub_positions, position_names(),
                     &q_subindices);
    vector<int> v_subindices;
    GetCommonIndices(sub_velocities, velocity_names(),
                     &v_subindices);
    return KinematicsSlice(tree(), q_subindices, v_subindices);
  }

 private:
  TreePtr tree_;
  InstanceIdMap instance_id_map_;
  ReverseIdMap instance_name_map_;
  KinematicsState initial_state_;
  vector<string> position_names_;
  vector<string> velocity_names_;
};

/**
 * Contains information regarding formulation of tracking problem.
 */
// TODO(eric.cousineau): Consider just placing equality constraints rather than
// removing the non-estimated joints.
class DartFormulation {
 public:
  struct Param {
    vector<string> estimated_positions;
    vector<string> estimated_velocities;
  };

  DartFormulation(unique_ptr<DartScene> scene, const Param& param)
      : param_(param),
        scene_(std::move(scene)) {
    const auto& q_slice = kinematics_slice_->q();
    const auto& v_slice = kinematics_slice_->v();

    // Add kinematics variables.
    kinematics_vars_.q() =
        prog_.NewContinuousVariables(q_slice.size(), param_.estimated_positions);
    kinematics_vars_.v() =
        prog_.NewContinuousVariables(v_slice.size(), param_.estimated_velocities);
  }

  const KinematicsSlice& kinematics_slice() const { return *kinematics_slice_; }
  const VectorSlice& q_slice() const { return kinematics_slice_->q(); }

  /**
   * This is a VERY relaxed interface for objectives to access.
   * They should NOT attempt to call solve.
   * Additionally, no attempt should be made to obtain the prior solution state.
   * Rather, this will be passed in via DartObjective::Update().
   */
  MathematicalProgram& prog() {
    return prog_;
  }
  const RigidBodyTreed& tree() const {
    return scene_->tree();
  }
private:
  Param param_;
  unique_ptr<DartScene> scene_;
  MathematicalProgram prog_;
  KinematicsVars kinematics_vars_;
  unique_ptr<KinematicsSlice> kinematics_slice_;
};

/**
 * An QP EKF objective may have induce both costs and constraints.
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
  // TODO(eric.cousineau): See if there is a way to remove this.
  static constexpr double kInitialUncorrelatedVariance = 1e-10;

  DartObjective(DartFormulation* formulation)
      : formulation_(formulation) {}

  /**
   * Initialize the objective, adding costs and/or constraints.
   */
  virtual void Init() = 0;

  virtual string name() const {
    return NiceTypeName::Get(*this);
  }

  virtual const OptVars& opt_vars() const = 0;

  virtual bool RequiresObservation() const {
    return true;
  }

  virtual void UpdateFormulation(
      double t,
      const KinematicsCached* cache_prior,
      const VectorXd& obj_priors) = 0;

  /**
   * Each individual subclass must implement its own Observe() functionality
   * to take in (a) time and (b) measurement data (joint states, image, etc.).
   */
  double latest_observation_time() const {
    return latest_observation_time_;
  }

  // TODO(eric.cousineau): Figure out more elegance for initial covariance.
  virtual MatrixXd GetInitialCovariance() const {
    // TODO(eric.cousineau): See if there is a better way to do this, per
    // Greg's comments.
    const int num_vars = opt_vars().size();
    return kInitialUncorrelatedVariance *
        MatrixXd::Identity(num_vars, num_vars);
  }
 protected:
  void set_latest_observation_time(double time) {
    latest_observation_time_ = time;
  }
  DartFormulation& formulation() const { return *formulation_; }
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
  struct Param {
    double max_observed_time_diff {1e-4};
  };

  DartEstimator(unique_ptr<DartFormulation> formulation, const Param& param)
    : param_(param),
      formulation_(std::move(formulation)),
      cache_prior_(formulation_->tree().CreateKinematicsCache()) {
    // Add kinematics variables.
  }

  void Update(double t) {
    // Get prior solution.
    MathematicalProgram& prog = this->prog();
    VectorXd sol_prior_ = prog.GetSolution();

    for (auto objective : objectives_) {
      // Ensure that this objective has had a useful observation.
      if (objective->RequiresObservation()) {
        double obj_obs_time = objective->latest_observation_time();
        double time_diff = t - obj_obs_time;
        ASSERT_THROW_FMT(time_diff >= 0,
                         "Observation in the future.\n"
                         "Update time: {}; Obs: {{name: {}, t: {}}}",
                         t, objective->name(), obj_obs_time);

        ASSERT_THROW_FMT(time_diff < param_.max_observed_time_diff,
                         "Observation too old.\n"
                         "Update time: {}, Obs: {{name: {}, t: {}}}",
                         t, objective->name(), obj_obs_time);
      }
      // Get the prior from the previous solution.

      objective->UpdateFormulation(t, formulation_.get());
    }
    formulation_.Solve();
  }

 protected:
  const RigidBodyTreed& tree() const { return formulation_->tree(); }
  MathematicalProgram& prog() { return formulation_->prog(); }
 private:
  Param param_;
  unique_ptr<DartFormulation> formulation_;
  DartObjectiveList objectives_;
  // Track which values are use for the optimization.
  vector<VectorSlice> opt_var_slices_;

  KinematicsState state_prior_;
  KinematicsCached cache_prior_;
};

}  // manipulation
}  // drake

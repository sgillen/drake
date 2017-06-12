#include "drake/manipulation/estimators/dev/dart.h"

namespace drake {
namespace manipulation {

DartScene::DartScene(TreePtr tree, const InstanceIdMap& instance_id_map)
  : tree_(tree),
    instance_id_map_(instance_id_map) {
  instance_name_map_ = ReverseMap(instance_id_map);
  GetHierarchicalKinematicNameList(this->tree(), instance_name_map_,
                                   &position_names_, &velocity_names_);
}

KinematicsSlice DartScene::CreateKinematicsSlice(
    const vector<string>& sub_positions,
    const vector<string>& sub_velocities) const {
  // Get slices.
  vector<int> q_subindices;
  GetSubIndices(sub_positions, position_names(),
                &q_subindices);
  vector<int> v_subindices;
  GetSubIndices(sub_velocities, velocity_names(),
                &v_subindices);
  return KinematicsSlice(tree(), q_subindices, v_subindices);
}

DartFormulation::DartFormulation(unique_ptr<DartScene> scene,
                                 const DartFormulation::Param& param)
  : param_(param),
    scene_(std::move(scene)),
    kinematics_est_slice_(
      scene_->CreateKinematicsSlice(param_.estimated_positions,
                                    param_.estimated_velocities)),
    kinematics_nonest_slice_(kinematics_est_slice_.Inverse()) {
  const auto& q_slice = kinematics_est_slice_.q();
  const auto& v_slice = kinematics_est_slice_.v();

  // Add kinematics variables.
  kinematics_est_vars_.q() =
      prog_.NewContinuousVariables(q_slice.size(), param_.estimated_positions);
  kinematics_est_vars_.v() =
      prog_.NewContinuousVariables(v_slice.size(), param_.estimated_velocities);
}

void DartFormulation::AddObjective(unique_ptr<DartObjective> objective) {
  // Allow objective to initialize values.
  objectives_.push_back(std::move(objective));
  auto& final = objectives_.back();
  final->Init();
}

DartEstimator::DartEstimator(unique_ptr<DartFormulation> formulation,
                             const DartEstimator::Param& param)
  : param_(param),
    formulation_(std::move(formulation)),
    state_prior_(formulation_->tree()),
    state_prior_with_input_(formulation_->tree()),
    cache_prior_with_input_(formulation_->tree().CreateKinematicsCache()),
    state_meas_(formulation_->tree()) {
  // Add kinematics variables.
}

void DartEstimator::Compile() {
  formulation_->Compile();
  // Aggregate covariance.
  int num_var = prog().num_vars();
  covariance_.resize(num_var, num_var);
  covariance_.setZero();
  int var_index = 0;
  // Add joint covariance.
  for (auto& objective : objectives()) {
    // Aggregate covariance.
  }
}

void DartEstimator::ObserveAndInputKinematicsState(
    double t, const KinematicsState& state_meas) {
  latest_state_observation_time_ = t;
  // Update measurement, v_q{k}, where t{k} = t.
  // TODO(eric.cousineau): Be more precise with sizing here...
  state_meas_ = state_meas;

  // Set input in the appropriate places.
  const auto& nonest_slice = formulation_->kinematics_nonest_slice();
  auto state_sub_nonest = nonest_slice.CreateFromSuperset(state_meas_);
  state_prior_with_input_ = state_prior_;
  nonest_slice.WriteToSuperset(state_sub_nonest, state_prior_with_input_);
}

const KinematicsState& DartEstimator::Update(double t) {
  // Get prior solution.
  MathematicalProgram& prog = this->prog();

  CheckObservationTime(t, latest_state_observation_time_, "update");

  int i = 0;
  for (auto& objective : objectives()) {
    // Ensure that this objective has had a useful observation.
    if (objective->RequiresObservation()) {
      double obj_obs_time = objective->latest_observation_time();

    }

    // Get the prior from the previous solution.
    const VectorSlice& slice = formulation_->objective_var_slice(i);
    objective->UpdateFormulation(t, formulation_.get());

    i++;
  }
  formulation_.Solve();
}

void DartEstimator::CheckObservationTime(double t_now, double t_obs,
                                         const string& name) const {
  double time_diff = t_now - t_obs;
  ASSERT_THROW_FMT(time_diff >= 0,
                   "Observation in the future.\n"
                   "Update time: {}; Obs: {{name: {}, t: {}}}",
                   t_now, name, t_obs);

  ASSERT_THROW_FMT(time_diff < param_.max_observed_time_diff,
                   "Observation too old.\n"
                   "Update time: {}, Obs: {{name: {}, t: {}}}",
                   t_now, name, t_obs);
}



}  // manipulation
}  // drake

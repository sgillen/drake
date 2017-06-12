#include "drake/manipulation/estimators/dev/dart.h"

namespace drake {
namespace manipulation {

DartScene::DartScene(TreePtr tree, const KinematicsState& initial_state, const InstanceIdMap& instance_id_map)
  : tree_(tree),
    initial_state_(initial_state),
    instance_id_map_(instance_id_map) {
  instance_name_map_ = ReverseMap(instance_id_map);
  GetHierarchicalKinematicNameList(this->tree(), instance_name_map_,
                                   &position_names_, &velocity_names_);
}

KinematicsSlice DartScene::CreateKinematicsSlice(const vector<string>& sub_positions, const vector<string>& sub_velocities) const {
  // Get slices.
  vector<int> q_subindices;
  GetSubIndices(sub_positions, position_names(),
                &q_subindices);
  vector<int> v_subindices;
  GetSubIndices(sub_velocities, velocity_names(),
                &v_subindices);
  return KinematicsSlice(tree(), q_subindices, v_subindices);
}

DartFormulation::DartFormulation(unique_ptr<DartScene> scene, const DartFormulation::Param& param)
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

void DartFormulation::AddObjective(unique_ptr<DartObjective> objective) {
  // Allow objective to initialize values.
  objectives_.push_back(std::move(objective));
  auto& final = objectives_.back();
  final->Init();
}

void DartEstimator::Update(double t) {
  // Get prior solution.
  MathematicalProgram& prog = this->prog();

  int i = 0;
  for (auto& objective : objectives_) {
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

    i++;
  }
  formulation_.Solve();
}



}  // manipulation
}  // drake

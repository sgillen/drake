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

  // Add kinematics variables first.
  kinematics_est_vars_.q() =
      prog_.NewContinuousVariables(q_slice.size(), param_.estimated_positions);
  kinematics_est_vars_.v() =
      prog_.NewContinuousVariables(v_slice.size(), param_.estimated_velocities);
}

void DartFormulation::AddObjective(unique_ptr<DartObjective> objective) {
  // Register objective, but do not initialize it.
  // Defer to DartEstimator to initialize, as this will have the initial state.
  objectives_.push_back(std::move(objective));
}

VectorSlice GetSubSliceVar(const OptVars& a, const OptVars& b) {
  return GetSubSlice(MakeIterableMatrix(a), MakeIterableMatrix(b));
}

void DartFormulation::PostInit() {
  DRAKE_DEMAND(!initialized_);
  initialized_ = true;
  // Initialize state slices.
  // TODO(eric.cousineau): Figure out better way to do this.
  OptVars all_vars = prog().decision_variables();
  kinematics_est_var_slice_ =
      KinematicsSlice(GetSubSliceVar(kinematics_est_vars_.q(), all_vars),
                      GetSubSliceVar(kinematics_est_vars_.v(), all_vars));

  // TODO(eric.cousineau): Make assumptions on variable ordering more
  // consistent. This permits ragged variable access, while below, it does
  // block-level access.
  int num_vars_sum = kinematics_est_var_slice_.size();
  for (auto& objective : objectives_) {
    OptVars obj_vars = objective->GetVars();
    objective_var_slices_.push_back(GetSubSliceVar(obj_vars, all_vars));
    num_vars_sum += obj_vars.size();
  }
  // Sanity check.
  DRAKE_DEMAND(objective_var_slices_.size() == objectives_.size());
  ASSERT_THROW_FMT(num_vars_sum == all_vars.size(),
                   "{} != {}", num_vars_sum, all_vars.size());
}

DartEstimator::DartEstimator(unique_ptr<DartFormulation> formulation,
                             const DartEstimator::Param& param)
  : param_(param),
    formulation_(std::move(formulation)),
    state_meas_(formulation_->tree()),
    state_prior_with_input_(formulation_->tree()),
    cache_prior_with_input_(formulation_->tree().CreateKinematicsCache()) {
  Compile();
}

void DartEstimator::Compile() {
  // Initialize each objective, making it aware of the initial kinematic state.
  state_prior_with_input_ = param_.initial_state;
  UpdateKinematicsWithPriorAndInput();

  for (auto& objective : objectives()) {
    objective->Init(cache_prior_with_input_);
  }

  // Ensure that the formulation is aware of where each objective is placed.
  formulation_->PostInit();

  // Aggregate covariance and initial conditions.
  int num_var = prog().num_vars();
  opt_val_prior_.resize(num_var);
  covariance_.resize(num_var, num_var);
  covariance_.setZero();

  // Simple iteration.
  int sub_index = 0;
  int sub_num_var = 0;
  auto next_sub = [&](int sub_num_var_next) {
    // Advance and prep for the next block.
    sub_index += sub_num_var;
    sub_num_var = sub_num_var_next;
  };
  auto get_covar_sub = [&]() {
    return covariance_.block(sub_index, sub_index, sub_num_var, sub_num_var);
  };
  auto get_init_sub = [&]() {
    return opt_val_prior_.segment(sub_index, sub_num_var);
  };

  // Add joint information.
  const auto& est_slice = formulation_->kinematics_est_slice();
  auto est_initial_state = est_slice.CreateFromSuperset(param_.initial_state);

  int nq = est_slice.q().size();
  next_sub(nq);
  get_init_sub() = est_initial_state.q();
  get_covar_sub() = CreateDefaultCovarianceMatrix(nq);

  int nv = est_slice.v().size();
  next_sub(nv);
  get_init_sub() = est_initial_state.v();
  get_covar_sub() = CreateDefaultCovarianceMatrix(nv);

  for (auto& objective : objectives()) {
    // Aggregate covariance.
    int obj_num_var = objective->GetVars().rows();
    next_sub(obj_num_var);
    get_init_sub() = objective->GetInitialValues();
    get_covar_sub() = objective->GetInitialCovariance();
  }

  // Check final size.
  next_sub(0);
  DRAKE_DEMAND(sub_index == num_var);
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
  nonest_slice.WriteToSuperset(state_sub_nonest, state_prior_with_input_);
}

namespace {
void ComputeQPHessian(const MathematicalProgram& prog,
                      MatrixXd* pH) {
  // More efficient way to do this?
  // Create QP solver directly, and then query the Hessian after?
  MatrixXd& H = *pH;

  int num_vars = prog.num_vars();
  H.resize(num_vars, num_vars);
  H.setZero();

  // Snagged from EqualityConstrainedQPSolver::Solve(...)
  // TODO(eric.cousineau): Use index slicing, if possible?
  DRAKE_ASSERT(prog.generic_constraints().empty());
  DRAKE_ASSERT(prog.generic_costs().empty());
  DRAKE_ASSERT(prog.linear_constraints().empty());
  DRAKE_ASSERT(prog.bounding_box_constraints().empty());
  DRAKE_ASSERT(prog.linear_complementarity_constraints().empty());

  for (auto const& binding : prog.quadratic_costs()) {
    const auto& Q = binding.constraint()->Q();
    int num_v_variables = binding.variables().rows();
    std::vector<size_t> v_index(num_v_variables);
    for (int i = 0; i < num_v_variables; ++i) {
      v_index[i] = prog.FindDecisionVariableIndex(binding.variables()(i));
    }
    for (int i = 0; i < num_v_variables; ++i) {
      for (int j = 0; j < num_v_variables; ++j) {
        H(v_index[i], v_index[j]) += Q(i, j);
      }
    }
  }
}
}  // namespace

const KinematicsState& DartEstimator::Update(double t) {
  // Set initial guess to prior solution.
  MathematicalProgram& prog = this->prog();
  prog.SetInitialGuessForAllVariables(opt_val_prior_);
  ASSERT_THROW_FMT(!has_nan(opt_val_prior_),
                   "Initial guess has nans:\n{}", opt_val_prior_.transpose());

  CheckObservationTime(t, latest_state_observation_time_, "update");

  // Update kinematics.
  UpdateKinematicsWithPriorAndInput();

  // Update each objective.
  int i = 0;
  for (auto& objective : objectives()) {
    // Ensure that this objective has had a useful observation.
    if (objective->RequiresObservation()) {
      CheckObservationTime(
          t, objective->latest_observation_time(), objective->name());
    }

    // Get the prior from the previous solution.
    const VectorSlice& obj_var_slice = formulation_->objective_var_slice(i);
    VectorXd obj_prior(obj_var_slice.size());
    obj_var_slice.ReadFromSuperset(opt_val_prior_, obj_prior);

    objective->UpdateFormulation(t, cache_prior_with_input_, obj_prior);
    i++;
  }

  // Solve.
  auto result = prog.Solve();
  DRAKE_ASSERT(result == kSolutionFound);

  // Update covariance, since all quadratic costs will include the Hessian.
  int num_vars = prog.num_vars();
  MatrixXd H(num_vars, num_vars);
  ComputeQPHessian(prog, &H);
  // TODO(eric.cousineau): Ensure that H is well-conditioned.
  covariance_ << H.inverse();

  // Get solution.
  // Using `prior`, because after this, it will become the prior.
  opt_val_prior_ = prog.GetSolutionVectorValues();

  ASSERT_THROW_FMT(!has_nan(opt_val_prior_),
                   "Solution has nans:\n{}", opt_val_prior_.transpose());

  // Update estimated states.
  auto& state_est_var_slice = formulation_->kinematics_est_var_slice();
  auto state_est_sub = state_est_var_slice.CreateZero<KinematicsState>();
  state_est_var_slice.ReadFromVectorSuperset(opt_val_prior_, state_est_sub);

  // Update prior with new estimates.
  // TODO(eric.cousineau): Determine if there should just be one place for
  // mixing priors with inputs.
  auto& state_est_slice = formulation_->kinematics_est_slice();
  state_est_slice.WriteToSuperset(state_est_sub, state_prior_with_input_);

  return state_prior_with_input_;
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

void DartEstimator::UpdateKinematicsWithPriorAndInput() {
  cache_prior_with_input_.initialize(state_prior_with_input_.q(),
                                     state_prior_with_input_.v());
  tree().doKinematics(cache_prior_with_input_);
}

MatrixXd CreateDefaultCovarianceMatrix(int num_vars,
                                       double initial_uncorrelated_variance) {
  // TODO(eric.cousineau): See if there is a better way to do this, per
  // Greg's comments.
  return initial_uncorrelated_variance *
      MatrixXd::Identity(num_vars, num_vars);
}

}  // manipulation
}  // drake

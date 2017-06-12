#include "drake/manipulation/estimators/dev/dart_objectives.h"

namespace drake {
namespace manipulation {

class DartJointObjective::Impl {
public:
  struct Cache {
    // Input variables.
    MatrixXd W;
    VectorXd q_est_meas;
    // Output variables.
    MatrixXd Q;
    MatrixXd b;
    double c;

    void Init(int nq) {
      W.resize(nq, nq);
      W.setZero();
      q_est_meas.resize(nq);
      q_est_meas.setZero();
      Update();
    }

    void Update() {
      // Update given that W and q_est_meas has been upated.
      // See MakeQuadraticErrorCost
      Q = 2 * W;
      b = -2 * Q * q_est_meas;
      c = q_est_meas.dot(W * q_est_meas);
    }
  };
  Cache cache_;
};

DartJointObjective::DartJointObjective(
    DartFormulation* formulation_,
    const Param& param)
  : DartObjective(formulation_),
    param_(param) {
  DRAKE_DEMAND(param_.joint_variance.size() == q_est_vars().size());
  impl_.reset(new Impl());
}

void DartJointObjective::Init(const KinematicsCached& kin_cache) {
  unused(kin_cache);
  KinematicsState meas_state(kin_cache);
  KinematicsState est_meas_state =
      formulation().kinematics_est_slice().CreateFromSuperset(meas_state);

  // Initialize for penalties on estimated joint positions.
  Impl::Cache& cache = impl_->cache_;
  int nq = q_est_vars().size();
  cache.Init(nq);
  cache.W =
      ComputeWeight(param_.joint_variance.array()).matrix().asDiagonal();
  cache.q_est_meas = est_meas_state.q();
  cache.Update();
  cost_ = make_shared<QuadraticCost>(cache.Q, cache.b, cache.c);
  prog().AddCost(cost_, q_est_vars());
}

void DartJointObjective::UpdateFormulation(double t,
                                           const KinematicsCached& kin_cache,
                                           const VectorXd& obj_prior) {
  KinematicsState meas_state(kin_cache);
  KinematicsState est_meas_state =
      formulation().kinematics_est_slice().CreateFromSuperset(meas_state);
  // Simply update the cost.
  Impl::Cache& cache = impl_->cache_;
  cache.q_est_meas = est_meas_state.q();
  cache.Update();
  cost_->UpdateCoefficients(cache.Q, cache.b, cache.c);
}


}  // namespace manipulation
}  // namespace drake

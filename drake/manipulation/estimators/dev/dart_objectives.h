#pragma once

#include "drake/manipulation/estimators/dev/dart.h"

namespace drake {
namespace manipulation {

// TODO(eric.cousineau): Clearly delineate between variance and virtual variance
// (weight).
template <typename T>
auto ComputeWeight(const T& variance) {
  // NOTE: Test with ArrayXd.
  return 1 / (2 * pow(variance, 2));
}

class DartJointObjective : public DartObjective {
 public:
  struct Param {
    VectorXd joint_variance;
  };

  DartJointObjective(DartFormulation* formulation_, const Param& param);
  ~DartJointObjective();

  void Init(const KinematicsCached& cache) override;
  void UpdateFormulation(double t, const KinematicsCached& kin_cache,
                         const VectorXd &obj_prior) override;
  const OptVars& GetVars() const override { return extra_vars_; }
  const VectorXd& GetInitialValues() const override { return extra_ic_; }
  void ObserveState(double t, const KinematicsState& state_meas);
 private:
  shared_ptr<QuadraticCost> cost_;
  Param param_;
  class Impl;
  unique_ptr<Impl> impl_;
  // Throw-away?
  VectorXd extra_ic_;
  OptVars extra_vars_;
};

}  // namespace manipulation
}  // namespace drake

#pragma once

#include <limits>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"

#include "drake/lcm/drake_lcm.h"

namespace drake {
namespace manipulation {

/**
 * Simple mixin to get simplified aliases.
 */
template <typename T_>
class LeafSystemMixin : public systems::LeafSystem<T_> {
 public:
  typedef T_ T;
  typedef systems::Context<T> Context;
  typedef systems::DiscreteValues<T> DiscreteValues;
  typedef systems::SystemOutput<T> SystemOutput;
  using Inport = systems::InputPortDescriptor<T>;
  using Outport = systems::OutputPortDescriptor<T>;
  template <typename U>
  using Value = systems::Value<U>;
};

class ArticulatedStateEstimator : public LeafSystemMixin<double> {
 public:
  ArticulatedStateEstimator(const std::string& config_file);

  void DoCalcDiscreteVariableUpdates(const Context& context,
                                     DiscreteValues* updates) const override;
  void DoCalcOutput(
      const Context& context, SystemOutput* output) const override;

  const Inport& inport_point_cloud() const;
  const Inport& inport_depth_image() const;
  const Inport& inport_tree_q_measurement() const;
  const Outport& outport_tree_state_estimate() const;

private:
  class Impl;
  std::shared_ptr<Impl> impl_;
  friend class Impl;

  int param_q0_index_{};

  int inport_point_cloud_index_{};
  int inport_depth_image_index_{};
  int inport_tree_q_measurement_index_{};

  int state_tree_state_estimate_index_{};
  int outport_tree_state_estimate_index_{};
};

}  // namespace manipulation
}  // namespace drake

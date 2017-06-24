#pragma once

#include <memory>

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/framework/diagram.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

class PerceptionHack {
 public:
  using DiagramBuilder = drake::systems::DiagramBuilder<double>;
  using TreePlant = IiwaAndWsgPlantWithStateEstimator<double>;
  using DrakeLcm = drake::lcm::DrakeLcm;
  PerceptionHack() {}
  void Inject(DiagramBuilder* pbuilder, DrakeLcm* plcm, TreePlant* pplant);
  ~PerceptionHack();
 private:
  class Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

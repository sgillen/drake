#pragma once

#include <memory>

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/framework/diagram.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"
#include "drake/manipulation/estimators/dev/articulated_state_estimator.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

using manipulation::ReverseIdMap;

class PerceptionHack {
 public:
  using DiagramBuilder = drake::systems::DiagramBuilder<double>;
  using TreePlant = IiwaAndWsgPlantWithStateEstimator<double>;
  using DrakeLcm = drake::lcm::DrakeLcm;
  PerceptionHack() {}
  void Inject(DiagramBuilder* pbuilder, DrakeLcm* plcm, TreePlant* pplant,
              const ReverseIdMap& plant_id_map);
  systems::DrakeVisualizer* GetEstimationVisualizer();
  ~PerceptionHack();
 private:
  class Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

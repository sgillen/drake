#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant_sdf.h"

#include <memory>

#include "drake/common/find_resource.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace acrobot {

using Eigen::Vector3d;

using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::UniformGravityFieldElement;

std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeAcrobotPlantSdf() {
  auto plant = std::make_unique<MultibodyPlant<double>>();

  parsing::AddModelFromSdfFile(
      FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.sdf"),
      plant.get());

  // Gravity acting in the -z direction.
  // TODO(nkoenig) A model in SDF has no knowledge about external forces,
  // such as gravity. We would have to place the model inside a <world>
  // and assign a <gravity>.
  plant->AddForceElement<UniformGravityFieldElement>(
      -9.81 * Vector3d::UnitZ());

  plant->Finalize();

  return plant;
}

}  // namespace acrobot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake

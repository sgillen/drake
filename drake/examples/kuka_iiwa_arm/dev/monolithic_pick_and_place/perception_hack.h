#pragma once

#include <memory>
#include <utility>

#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

const char kIiwaUrdfMesh[] =
    "/manipulation/models/iiwa_description/urdf/"
    "iiwa14_mesh_collision.urdf";

/*
 * @param use_slow_meshes Use slower mesh versions (for depth sensor
 * simulation).

    bool use_slow_meshes = false) {
  tree_builder->StoreModel("iiwa", use_slow_meshes ? kIiwaUrdfMesh : kIiwaUrdf);
  const char kTable[] = "/examples/kuka_iiwa_arm/models/table/"
                        "extra_heavy_duty_table_surface_only_collision.sdf";
  const char kTableMesh[] =  "/examples/kuka_iiwa_arm/models/table/"
                             "extra_heavy_duty_table.sdf";
  tree_builder->StoreModel("table", use_slow_meshes ? kTableMesh : kTable);
*/

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

#include <memory>
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree_construction.h"

int main(int argc, char* argv[]) {
    // RigidBodyPlant<double>* plant = nullptr;
  const char* kModelPath = /*"drake/"*/
      "manipulation/models/iiwa_description/"
      "urdf/iiwa14_polytope_collision.urdf";
  const std::string urdf =
      kModelPath;
      // (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kModelPath));
  // return 10;  // exits 10

  auto tree = std::make_unique<RigidBodyTree<double>>();
  // return 10;  // exits 135

  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf, drake::multibody::joints::kFixed, tree.get());
  return 10;  // dies with stacktrace, `corrupted size vs. prev_size`
}

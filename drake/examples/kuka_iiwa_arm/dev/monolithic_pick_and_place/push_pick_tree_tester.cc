/**
 * @file test demo to visualize a push demo tree with the book in a random set of configurations.
 */
#include <chrono>
#include <thread>

#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/simple_tree_visualizer.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/manipulation/util/simple_tree_visualizer.h"
//
//DEFINE_int32(num_configurations, 10,
//"Number of random test configurations to display in the demo");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place_demo {
using manipulation::util::ModelInstanceInfo;
using manipulation::util::WorldSimTreeBuilder;
using manipulation::util::S;

// Adds a demo tree.
const char* const kModelPath =
    "drake/manipulation/models/iiwa_description/urdf/"
        "iiwa14_polytope_collision.urdf";


std::unique_ptr<RigidBodyTreed> BuildDemoTree(
    ModelInstanceInfo<double>* iiwa_instance, ModelInstanceInfo<double>* wsg_instance,
    ModelInstanceInfo<double>* box_instance) {
auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

// Adds models to the simulation builder. Instances of these models can be
// subsequently added to the world.
tree_builder->StoreModel("iiwa", kModelPath);
tree_builder->StoreModel("table",
"drake/examples/kuka_iiwa_arm/models/table/"
"extra_heavy_duty_table_surface_only_collision.sdf");
tree_builder->StoreModel("box",
"drake/examples/kuka_iiwa_arm/models/objects/"
"block_for_pick_and_place.urdf");
tree_builder->StoreModel(
"wsg",
"drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf");

// Build a world with two fixed tables.  A box is placed one on
// table, and the iiwa arm is fixed to the other.
tree_builder->AddFixedModelInstance("table",
Eigen::Vector3d::Zero() /* xyz */,
    Eigen::Vector3d::Zero() /* rpy */);
tree_builder->AddFixedModelInstance("table",
Eigen::Vector3d(0.8, 0, 0) /* xyz */,
Eigen::Vector3d::Zero() /* rpy */);
tree_builder->AddFixedModelInstance("table",
Eigen::Vector3d(0, 0.85, 0) /* xyz */,
Eigen::Vector3d::Zero() /* rpy */);

tree_builder->AddGround();

// The `z` coordinate of the top of the table in the world frame.
// The quantity 0.736 is the `z` coordinate of the frame associated with the
// 'surface' collision element in the SDF. This element uses a box of height
// 0.057m thus giving the surface height (`z`) in world coordinates as
// 0.736 + 0.057 / 2.
const double kTableTopZInWorld = 0.736 + 0.057 / 2;

// Coordinates for kRobotBase originally from iiwa_world_demo.cc.
// The intention is to center the robot on the table.
const Eigen::Vector3d kRobotBase(-0.243716, -0.625087, kTableTopZInWorld);
// Start the box slightly above the table.  If we place it at
// the table top exactly, it may start colliding the table (which is
// not good, as it will likely shoot off into space).
const Eigen::Vector3d kBoxBase(1 + -0.43, -0.65, kTableTopZInWorld + 0.1);

int id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
*iiwa_instance = tree_builder->get_model_info_for_instance(id);
id = tree_builder->AddFloatingModelInstance("box", kBoxBase,
                                            Vector3<double>(0, 0, 1));
*box_instance = tree_builder->get_model_info_for_instance(id);
id = tree_builder->AddModelInstanceToFrame(
    "wsg", tree_builder->tree().findFrame("iiwa_frame_ee"),
    drake::multibody::joints::kFixed);
*wsg_instance = tree_builder->get_model_info_for_instance(id);

  return tree_builder->Build();
}


int DoMain() {
  drake::lcm::DrakeLcm lcm;


  auto tree = std::make_unique<RigidBodyTree<double>>();

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
      Eigen::Vector3d::Zero() /* base position */,
      Eigen::Vector3d::Zero() /* base orientation */);

  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow(kModelPath), drake::multibody::joints::kFixed,
      weld_to_frame, tree.get());

  SimpleTreeVisualizer simple_tree_visualizer(*tree.get(), &lcm);

  // Simple demo that iterates through a bunch of joint configurations.
  for (int i = 0; i < FLAGS_num_configurations; ++i) {
    simple_tree_visualizer.visualize(Eigen::VectorXd::Random(7));

    // Sleep for a second just so that the new configuration can be seen
    // on the visualizer.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}

}  // monolithic_pick_and_place_demo
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::manipulation::DoMain();
}

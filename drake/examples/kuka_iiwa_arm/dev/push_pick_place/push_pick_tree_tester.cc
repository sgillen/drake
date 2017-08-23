/**
 * @file test demo to visualize a push demo tree with the book in a random set of configurations.
 */
#include <chrono>
#include <thread>

#include <gflags/gflags.h>
#include <stdlib.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/simple_tree_visualizer.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/manipulation/util/simple_tree_visualizer.h"

DEFINE_int32(num_configurations, 10,
"Number of random test configurations to display in the demo");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place_demo {
using manipulation::util::ModelInstanceInfo;
using manipulation::util::WorldSimTreeBuilder;
using manipulation::util::SimpleTreeVisualizer;

// Adds a demo tree.
const char* const kModelPath =
    "drake/manipulation/models/iiwa_description/urdf/"
        "iiwa14_polytope_collision.urdf";


std::unique_ptr<RigidBodyTreed> BuildDemoTree() {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel("iiwa", kModelPath);
  tree_builder->StoreModel("table",
  "drake/examples/kuka_iiwa_arm/models/table/"
  "extra_heavy_duty_table_surface_only_collision.sdf");
  tree_builder->StoreModel("book",
  "drake/examples/kuka_iiwa_arm/models/objects/"
  "book.urdf");
  tree_builder->StoreModel(
  "wsg",
  "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf");
  tree_builder->StoreModel(
      "white_table_top",
      "drake/examples/kuka_iiwa_arm/models/table/white_table_top.urdf");

  drake::log()->info("About to add tables");
  // Build a world with two fixed tables.  A box is placed one on
  // table, and the iiwa arm is fixed to the other.
  tree_builder->AddFixedModelInstance("table",
  Eigen::Vector3d::Zero() /* xyz */,
      Eigen::Vector3d::Zero() /* rpy */);

  // The `z` coordinate of the top of the table in the world frame.
  // The quantity 0.736 is the `z` coordinate of the frame associated with the
  // 'surface' collision element in the SDF. This element uses a box of height
  // 0.01m thus giving the surface height (`z`) in world coordinates as
  // 0.736 + 0.01 / 2.
  const double kTableTopZInWorld = 0.736 + 0.01 / 2;

  tree_builder->AddFixedModelInstance("white_table_top",
      Eigen::Vector3d(0.463, -0.843, kTableTopZInWorld + 0.011 ),
      Eigen::Vector3d::Zero());
  tree_builder->AddGround();


  // Coordinates for kRobotBase originally from iiwa_world_demo.cc.
  // The intention is to center the robot on the table.
  const Eigen::Vector3d kRobotBase(-0.243716, -0.625087, kTableTopZInWorld);
  // Start the box slightly above the table.  If we place it at
  // the table top exactly, it may start colliding the table (which is
  // not good, as it will likely shoot off into space).
  const Eigen::Vector3d kBoxBase(1 + -0.63, -0.65, kTableTopZInWorld + 0.03);

  drake::log()->info("About to add iiwa");
  tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
  drake::log()->info("About to add box");
  tree_builder->AddFloatingModelInstance("book", kBoxBase,
                                              Vector3<double>(0, 0, 1));
  drake::log()->info("About to add wsg");
  tree_builder->AddModelInstanceToFrame(
      "wsg", tree_builder->tree().findFrame("iiwa_frame_ee"),
      drake::multibody::joints::kFixed);

  drake::log()->info("About to build");
  return tree_builder->Build();
}


int DoMain() {
  drake::lcm::DrakeLcm lcm;

//
  std::unique_ptr<RigidBodyTreed> tree = BuildDemoTree();

  SimpleTreeVisualizer simple_tree_visualizer(*tree.get(), &lcm);

  drake::log()->info("tree num positions {}", tree->get_num_positions());
  // Simple demo that iterates through a bunch of joint configurations.
  for (int i = 0; i < FLAGS_num_configurations; ++i) {

    VectorX<double> positions = VectorX<double>::Zero(tree->get_num_positions());
    positions.segment<7>(0) = VectorX<double>::Random(7 /* IIWA */);
    //positions.segment<4>(10) = VectorX<double>::Random(4 /* box orientation */);

    double box_z = ((double)std::rand() / RAND_MAX) * M_PI;
    drake::log()->info("Box z {}", box_z);
    Eigen::Quaterniond box_pose_quat{Eigen::AngleAxis<double>(box_z, Eigen::Vector3d::UnitX())};
    positions.segment<4>(10) = box_pose_quat.coeffs();
    simple_tree_visualizer.visualize(positions);

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
  return drake::examples::kuka_iiwa_arm::monolithic_pick_and_place_demo::DoMain();
}

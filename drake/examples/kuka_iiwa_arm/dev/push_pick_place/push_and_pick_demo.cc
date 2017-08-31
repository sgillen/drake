#include <memory>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include "bot_core/robot_state_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/dev/push_pick_place/push_and_pick_state_machine_system.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_command.hpp"
#include "drake/manipulation/planner/robot_plan_interpolator.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_controller.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/manipulation/sensors/xtion.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
// -0.25* MP_PI
DEFINE_double(orientation,  -0.25*M_PI, "Yaw angle of the book.");
DEFINE_double(dt, 7.5e-4, "Integration step size");
DEFINE_double(realtime_rate, 0.5, "Rate at which to run the simulation, "
"relative to realtime");
DEFINE_bool(with_camera, true, "Attach an Asus Xtion to the gripper.");

using robotlocomotion::robot_plan_t;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace push_and_pick {
namespace {
using manipulation::schunk_wsg::SchunkWsgController;
using manipulation::schunk_wsg::SchunkWsgStatusSender;
using systems::RigidBodyPlant;
using systems::RungeKutta2Integrator;
using systems::Simulator;
using manipulation::util::ModelInstanceInfo;
using manipulation::planner::RobotPlanInterpolator;
using manipulation::util::WorldSimTreeBuilder;
using manipulation::sensors::Xtion;

const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
        "iiwa14_polytope_collision.urdf";
const char kIiwaEndEffectorName[] = "iiwa_link_ee";

// The `z` coordinate of the top of the table in the world frame.
// The quantity 0.736 is the `z` coordinate of the frame associated with the
// 'surface' collision element in the SDF. This element uses a box of height
// 0.057m thus giving the surface height (`z`) in world coordinates as
// 0.736 + 0.057 / 2.
const double kTableTopZInWorld = 0.736 + 0.057 / 2;

// Coordinates for kRobotBase originally from iiwa_world_demo.cc.
// The intention is to center the robot on the table.
// TODO(sam.creasey) fix this
//const Eigen::Vector3d kRobotBase(0, 0, kTableTopZInWorld);
const Eigen::Vector3d kRobotBase(-0.243716, -0.625087, kTableTopZInWorld);
const Eigen::Vector3d kTableBase(0.243716, 0.625087, 0.);

// Start the box slightly above the table.  If we place it at
// the table top exactly, it may start colliding the table (which is
// not good, as it will likely shoot off into space).
const Eigen::Vector3d kBookBase(1 + -0.63,-0.65 , kTableTopZInWorld + 0.03);
//const Eigen::Vector3d kBookBase(0.228,-0.27 , kTableTopZInWorld + 0.03);


std::unique_ptr<systems::RigidBodyPlant<double>> BuildCombinedPlant(
    ModelInstanceInfo<double>* iiwa_instance,
    ModelInstanceInfo<double>* wsg_instance,
    ModelInstanceInfo<double>* book_instance,
    const Eigen::Vector3d& book_position,
    const Eigen::Vector3d& book_orientation,
    Xtion** pcamera = nullptr,
    ModelInstanceInfo<double>* camera_instance = nullptr) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel("iiwa", kIiwaUrdf);
  tree_builder->StoreModel("table",
                           "drake/examples/kuka_iiwa_arm/models/table/"
                               "extra_heavy_duty_table_surface_only_collision.sdf");
  tree_builder->StoreModel("book",
                           "drake/examples/kuka_iiwa_arm/models/objects/"
                               "book.urdf");
//  tree_builder->StoreModel("book",
//                           "drake/examples/kuka_iiwa_arm/models/objects/"
//                               "black_box.urdf");
  tree_builder->StoreModel(
      "wsg",
      "drake/manipulation/models/wsg_50_description"
          "/sdf/schunk_wsg_50_ball_contact.sdf");
  tree_builder->StoreModel(
      "white_table_top",
      "drake/examples/kuka_iiwa_arm/models/table/white_table_top.urdf");

  drake::log()->info("AddfixedModelInstance");

  // The main table which the arm sits on.
  tree_builder->AddFixedModelInstance(
      "table", Eigen::Vector3d::Zero() /* xyz */,
      Eigen::Vector3d::Zero() /* rpy */);
  // The `z` coordinate of the top of the table in the world frame.
  // The quantity 0.736 is the `z` coordinate of the frame associated with the
  // 'surface' collision element in the SDF. This element uses a box of height
  // 0.01m thus giving the surface height (`z`) in world coordinates as
  // 0.736 + 0.01 / 2.

  drake::log()->info("AddfixedModelInstance white_table_top");
  tree_builder->AddFixedModelInstance("white_table_top",
                                      Eigen::Vector3d(0.463, -0.843,
                                      kTableTopZInWorld - 0.011 ),
                                      Eigen::Vector3d::Zero());
  tree_builder->AddGround();

  // Chooses an appropriate box.
  int box_id = 0;

  drake::log()->info("AddfixedModelInstance iiwa");

  int iiwa_id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
  *iiwa_instance = tree_builder->get_model_info_for_instance(iiwa_id);

  drake::log()->info("AddfixedModelInstance book");
  box_id = tree_builder->AddFloatingModelInstance("book", book_position,
                                                  book_orientation);
  *book_instance = tree_builder->get_model_info_for_instance(box_id);

  drake::log()->info("AddfixedModelInstance wsg");
  int wsg_id = tree_builder->AddModelInstanceToFrame(
      "wsg", tree_builder->tree().findFrame("iiwa_frame_ee"),
      drake::multibody::joints::kFixed);
  *wsg_instance = tree_builder->get_model_info_for_instance(wsg_id);

  if (pcamera) {
    // Attach Xtion (X) to WSG's end effector (G).
    Eigen::Isometry3d X_GX;
    X_GX.setIdentity();
    X_GX.linear() <<
        0, 1, 0,
        1, 0, 0,
        0, 0, -1;
    // clang-format on
    X_GX.translation() << 0, -0.015, -0.025;
    auto wsg_body = tree_builder->mutable_tree().FindBody("body", "", wsg_id);
    Xtion* camera = new Xtion(tree_builder.get(), wsg_body, X_GX);
    *pcamera = camera;
    DRAKE_DEMAND(camera_instance != nullptr);
    *camera_instance =
        tree_builder->get_model_info_for_instance(camera->instance_id());
  }

  return std::make_unique<systems::RigidBodyPlant<double>>(
      tree_builder->Build());
}


int DoMain(void) {
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;
  ModelInstanceInfo<double> iiwa_instance, wsg_instance, book_instance,
      camera_instance;
  Xtion* camera = nullptr;

  std::unique_ptr<systems::RigidBodyPlant<double>> model_ptr =
      BuildCombinedPlant(
          &iiwa_instance, &wsg_instance, &book_instance,
          kBookBase, Vector3<double>(0, 0, FLAGS_orientation),
          FLAGS_with_camera ? &camera : nullptr,
          &camera_instance);

  std::vector<ModelInstanceInfo<double>> ee_fixed_info = {camera_instance};
  auto plant =
      builder.AddSystem<IiwaAndWsgPlantWithStateEstimator<double>>(
      std::move(model_ptr), iiwa_instance, wsg_instance, book_instance,
      ee_fixed_info);
  plant->set_name("plant");
//
//  auto contact_viz =
//      builder.AddSystem<systems::ContactResultsToLcmSystem<double>>(
//          plant->get_tree());
//  auto contact_results_publisher = builder.AddSystem(
//      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
//          "CONTACT_RESULTS", &lcm));
//  // Contact results to lcm msg.
//  builder.Connect(plant->get_output_port_contact_results(),
//                  contact_viz->get_input_port(0));
//  builder.Connect(contact_viz->get_output_port(0),
//                  contact_results_publisher->get_input_port(0));

  auto drake_visualizer = builder.AddSystem<systems::DrakeVisualizer>(
      plant->get_plant().get_rigid_body_tree(), &lcm);

  builder.Connect(plant->get_output_port_plant_state(),
                  drake_visualizer->get_input_port(0));

  auto iiwa_trajectory_generator = builder.AddSystem<RobotPlanInterpolator>(
      FindResourceOrThrow(kIiwaUrdf));
  builder.Connect(plant->get_output_port_iiwa_state(),
                  iiwa_trajectory_generator->get_state_input_port());
  builder.Connect(
      iiwa_trajectory_generator->get_state_output_port(),
      plant->get_input_port_iiwa_state_command());
  builder.Connect(
      iiwa_trajectory_generator->get_acceleration_output_port(),
      plant->get_input_port_iiwa_acceleration_command());

  auto wsg_controller = builder.AddSystem<SchunkWsgController>();
  builder.Connect(plant->get_output_port_wsg_state(),
                  wsg_controller->get_state_input_port());
  builder.Connect(wsg_controller->get_output_port(0),
                  plant->get_input_port_wsg_command());

  auto wsg_status_sender = builder.AddSystem<SchunkWsgStatusSender>(
      plant->get_output_port_wsg_state().size(), 0, 0);
  builder.Connect(plant->get_output_port_wsg_state(),
                  wsg_status_sender->get_input_port(0));

  const Eigen::Vector3d robot_base = kRobotBase;//(0, 0, kTableTopZInWorld);
  Isometry3<double> iiwa_base = Isometry3<double>::Identity();
  iiwa_base.translation() = robot_base;

  auto state_machine =
      builder.template AddSystem<PushAndPickStateMachineSystem>(
          FindResourceOrThrow(kIiwaUrdf), kIiwaEndEffectorName,
          iiwa_base);

  // TODO(eric.cousineau): Replace this with an estimate from Jiaji's box
  // estimator.
  builder.Connect(plant->get_output_port_box_robot_state_msg(),
                  state_machine->get_input_port_box_state());

  builder.Connect(wsg_status_sender->get_output_port(0),
                  state_machine->get_input_port_wsg_status());
  builder.Connect(plant->get_output_port_iiwa_robot_state_msg(),
                  state_machine->get_input_port_iiwa_state());
  builder.Connect(state_machine->get_output_port_wsg_command(),
                  wsg_controller->get_command_input_port());
  builder.Connect(state_machine->get_output_port_iiwa_plan(),
                  iiwa_trajectory_generator->get_plan_input_port());

  // Add camera if enabled.
  if (camera) {
    camera->Build(&lcm, true, true);
    builder.AddSystem(std::unique_ptr<Xtion>(camera));
    builder.Connect(
        plant->get_output_port_plant_state(),
        camera->get_input_port_state());
  }

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);
  simulator.Initialize();
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.reset_integrator<RungeKutta2Integrator<double>>(*sys,
                                                            FLAGS_dt, simulator.get_mutable_context());
  simulator.get_mutable_integrator()->set_maximum_step_size(FLAGS_dt);
  simulator.get_mutable_integrator()->set_fixed_step_mode(true);

  auto& plan_source_context = sys->GetMutableSubsystemContext(
      *iiwa_trajectory_generator, simulator.get_mutable_context());
  iiwa_trajectory_generator->Initialize(
      plan_source_context.get_time(),
      Eigen::VectorXd::Zero(7),
      plan_source_context.get_mutable_state());

  drake::log()->info("Intended book position {}", kBookBase.transpose());

  // Step the simulator in some small increment.  Between steps, check
  // to see if the state machine thinks we're done, and if so that the
  // object is near the target.
  const double simulation_step = 0.1;
  while (state_machine->state(
      sys->GetSubsystemContext(*state_machine,
                               simulator.get_context()))
      != push_and_pick::kDone) {
    simulator.StepTo(simulator.get_context().get_time() + simulation_step);
  }
  return 0;
}

}  // namespace
}  // namespace push_and_pick
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::kuka_iiwa_arm::push_and_pick::DoMain();
}

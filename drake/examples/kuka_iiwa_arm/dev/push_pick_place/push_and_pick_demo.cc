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

DEFINE_double(orientation,  0.25*M_PI, "Yaw angle of the book.");
DEFINE_double(dt, 1e-3, "Integration step size");
DEFINE_double(realtime_rate, 0.0, "Rate at which to run the simulation, "
"relative to realtime");
//DEFINE_bool(quick, false, "Run only a brief simulation and return success "
//"without executing the entire task");

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
const Eigen::Vector3d kBookBase(1 + -0.63, -0.65, kTableTopZInWorld + 0.03);


std::unique_ptr<systems::RigidBodyPlant<double>> BuildCombinedPlant(
    ModelInstanceInfo<double>* iiwa_instance,
    ModelInstanceInfo<double>* wsg_instance,
    ModelInstanceInfo<double>* book_instance,
    const Eigen::Vector3d& book_position = kBookBase,
    const Eigen::Vector3d& book_orientation = Vector3<double>(0, 0, -1)
    ) {
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
                                      kTableTopZInWorld + 0.011 ),
                                      Eigen::Vector3d::Zero());
  tree_builder->AddGround();

  // Chooses an appropriate box.
  int box_id = 0;

  drake::log()->info("AddfixedModelInstance iiwa");

  int iiwa_id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
  *iiwa_instance = tree_builder->get_model_info_for_instance(iiwa_id);

  drake::log()->info("AddfixedModelInstance book");
  box_id = tree_builder->AddFloatingModelInstance("book", kBookBase,
                                                  book_orientation);
  *book_instance = tree_builder->get_model_info_for_instance(box_id);

  drake::log()->info("AddfixedModelInstance wsg");
  int wsg_id = tree_builder->AddModelInstanceToFrame(
      "wsg", tree_builder->tree().findFrame("iiwa_frame_ee"),
      drake::multibody::joints::kFixed);
  *wsg_instance = tree_builder->get_model_info_for_instance(wsg_id);

  return std::make_unique<systems::RigidBodyPlant<double>>(
      tree_builder->Build());
}


int DoMain(void) {
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;
  ModelInstanceInfo<double> iiwa_instance, wsg_instance, book_instance;

  std::unique_ptr<systems::RigidBodyPlant<double>> model_ptr =
      BuildCombinedPlant(
          &iiwa_instance, &wsg_instance, &book_instance,
          kBookBase, Vector3<double>(0, 0, FLAGS_orientation));

  auto plant = builder.AddSystem<IiwaAndWsgPlantWithStateEstimator<double>>(
      std::move(model_ptr), iiwa_instance, wsg_instance, book_instance);
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

  const Eigen::Vector3d robot_base(0, 0, kTableTopZInWorld);
  Isometry3<double> iiwa_base = Isometry3<double>::Identity();
  iiwa_base.translation() = robot_base;
//
//  if (FLAGS_end_position >= 0) {
//    if (FLAGS_end_position >= static_cast<int>(place_locations.size())) {
//      throw std::runtime_error("Invalid end position specified.");
//    }
//    std::vector<Isometry3<double>> new_place_locations;
//    new_place_locations.push_back(place_locations[FLAGS_end_position]);
//    place_locations.swap(new_place_locations);
//  }

  auto state_machine =
      builder.template AddSystem<PushAndPickStateMachineSystem>(
          FindResourceOrThrow(kIiwaUrdf), kIiwaEndEffectorName,
          iiwa_base);

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

  // Step the simulator in some small increment.  Between steps, check
  // to see if the state machine thinks we're done, and if so that the
  // object is near the target.
  const double simulation_step = 0.1;
  while (state_machine->state(
      sys->GetSubsystemContext(*state_machine,
                               simulator.get_context()))
      != push_and_pick::kDone) {
    simulator.StepTo(simulator.get_context().get_time() + simulation_step);
//    if (FLAGS_quick) {
//      // We've run a single step, just get out now since we won't have
//      // reached our destination.
//      return 0;
//    }
  }
//
//  const pick_and_place::WorldState& world_state =
//      state_machine->world_state(
//          sys->GetSubsystemContext(*state_machine,
//                                   simulator.get_context()));
//  const Isometry3<double>& object_pose = world_state.get_object_pose();
//  const Vector6<double>& object_velocity = world_state.get_object_velocity();
//  Isometry3<double> goal = place_locations.back();
//  goal.translation()(2) += kTableTopZInWorld;
//  Eigen::Vector3d object_rpy = math::rotmat2rpy(object_pose.rotation());
//  Eigen::Vector3d goal_rpy = math::rotmat2rpy(goal.rotation());
//
//  drake::log()->info("Pose: {} {}",
//                     object_pose.translation().transpose(),
//                     object_rpy.transpose());
//  drake::log()->info("Velocity: {}", object_velocity.transpose());
//  drake::log()->info("Goal: {} {}",
//                     goal.translation().transpose(),
//                     goal_rpy.transpose());
//
//  const double position_tolerance = 0.02;
//  Eigen::Vector3d position_error =
//      object_pose.translation() - goal.translation();
//  drake::log()->info("Position error: {}", position_error.transpose());
//  DRAKE_DEMAND(std::abs(position_error(0)) < position_tolerance);
//  DRAKE_DEMAND(std::abs(position_error(1)) < position_tolerance);
//  DRAKE_DEMAND(std::abs(position_error(2)) < position_tolerance);
//
//  const double angle_tolerance = 0.0873;  // 5 degrees
//  Eigen::Vector3d rpy_error = object_rpy - goal_rpy;
//  drake::log()->info("RPY error: {}", rpy_error.transpose());
//  DRAKE_DEMAND(std::abs(rpy_error(0)) < angle_tolerance);
//  DRAKE_DEMAND(std::abs(rpy_error(1)) < angle_tolerance);
//  DRAKE_DEMAND(std::abs(rpy_error(2)) < angle_tolerance);
//
//
//  const double linear_velocity_tolerance = 0.1;
//  DRAKE_DEMAND(std::abs(object_velocity(0)) < linear_velocity_tolerance);
//  DRAKE_DEMAND(std::abs(object_velocity(1)) < linear_velocity_tolerance);
//  DRAKE_DEMAND(std::abs(object_velocity(2)) < linear_velocity_tolerance);
//
//  const double angular_velocity_tolerance = 0.1;
//  DRAKE_DEMAND(std::abs(object_velocity(3)) < angular_velocity_tolerance);
//  DRAKE_DEMAND(std::abs(object_velocity(4)) < angular_velocity_tolerance);
//  DRAKE_DEMAND(std::abs(object_velocity(5)) < angular_velocity_tolerance);

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

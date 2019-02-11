#include <fstream>
#include <iostream>
#include <memory>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/frame_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/primitives/trajectory_source.h"


namespace hamr {
namespace {

using drake::multibody::joints::FloatingBaseType;
using drake::systems::ConstantVectorSource;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::DrakeVisualizer;
using drake::systems::FrameVisualizer;
using drake::systems::RigidBodyPlant;
using drake::systems::Simulator;
using drake::trajectories::PiecewisePolynomial;

int DoMain() {

  drake::lcm::DrakeLcm lcm;
  DiagramBuilder<double> builder;

  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      "tmp/hamr_scaled.urdf", drake::multibody::joints::kFixed,
      nullptr /* weld to frame */, tree.get());  

  auto hamr = builder.AddSystem<RigidBodyPlant<double>>(std::move(tree), 0.01);

  // Creates and adds LCM publisher for visualization.
  DrakeVisualizer* visualizer =
      builder.AddSystem<DrakeVisualizer>(hamr->get_rigid_body_tree(), &lcm);
  visualizer->set_publish_period(0.01);

  drake::VectorX<double> constant_vector =
      Eigen::MatrixXd::Zero(hamr->get_input_port(0).size(), 1);

  auto constant_zero_source =
      builder.AddSystem<ConstantVectorSource<double>>(constant_vector);
  constant_zero_source->set_name("zero input");

  // Connects the blank input command
  builder.Connect(constant_zero_source->get_output_port(),
                  hamr->get_input_port(0));
  builder.Connect(hamr->get_output_port(0), visualizer->get_input_port(0));

  auto sys = builder.Build();

  Simulator<double> simulator(*sys);

  lcm.StartReceiveThread();
  simulator.set_target_realtime_rate(100);
  simulator.set_publish_every_time_step(true);  

  // startPos();
  drake::VectorX<double> startPos =
      Eigen::MatrixXd::Zero(hamr->get_num_states(), 1);

  hamr->set_state_vector(&simulator.get_mutable_context(), startPos);
  simulator.Initialize();

  simulator.StepTo(50);

  return 0;
}

}  // namespace
}  // namespace hamr

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return hamr::DoMain();
}

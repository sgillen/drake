#include <gflags/gflags.h>
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
      "hamrfull/models/urdf/hamr_scaled.urdf", drake::multibody::joints::kFixed,
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

  // // robot properties
  // HamrURDFType urdf_type = kStandardHamr;
  // FloatingBaseType base_type = FloatingBaseType::kFixed;
  // double ustatic = 0.6;

  // // build hamr
  // Hamr* hamr = builder.AddSystem<Hamr>(urdf_type, base_type, dt, ustatic);

  // // actuator properties
  // int nact = 8;
  // std::vector<HamrActuatorType> act_type = {kOnePly, kOnePly, kOnePly,
  // kOnePly,
  //                                           kOnePly, kOnePly, kOnePly,
  //                                           kOnePly};
  // VectorX<double> orien(nact);
  // orien << -1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0;
  // VectorX<double> Vmax = Eigen::MatrixXd::Constant(nact, 1, 225);
  // VectorX<double> Vmin = Eigen::MatrixXd::Zero(nact, 1);

  // // Add hamr wth actuators diagram
  // auto hamr_w_act = builder.AddSystem<HamrWithActuators>(
  //     urdf_type, base_type, dt, ustatic, act_type, orien, Vmax, Vmin);
  // const Hamr& hamr = hamr_w_act->get_hamr();

  // Creates and adds LCM publisher for visualization
  // DrakeVisualizer* visualizer =
  //     builder.AddSystem<DrakeVisualizer>(hamr->get_rigid_body_tree(), &lcm);
  // visualizer->set_publish_period(dt);

  // set up signal loggers
  // auto state_logger =
  // builder.AddSystem<drake::systems::SignalLogger<double>>(
  //     hamr->get_num_states());
  // state_logger->set_publish_period(dt);

  // std::cout << "1" << std::endl;

  // // build input
  // // int Nt = ceil(T / 1.0) + 1;
  // // const double freq = 0.002;
  // // RowVectorX<double> tknot = Eigen::VectorXd::LinSpaced(Nt, 0,
  // // T).transpose(); VectorX<double> amp = Eigen::MatrixXd::Constant(nact, 1,
  // // 0); VectorX<double> offset = Eigen::MatrixXd::Constant(nact, 1, 0);
  // // Eigen::Matrix<double, 8, Eigen::Dynamic> fknot =
  // //     hamr_w_act->HamrSinusoidalInput(freq, tknot, amp, offset, kTrot);
  // // PiecewisePolynomial<double> ftraj =
  // //     PiecewisePolynomial<double>::FirstOrderHold(tknot, fknot);
  // // auto input =
  // //     builder.AddSystem<drake::systems::TrajectorySource<double>>(ftraj,
  // 0);
  // // std::cout << hamr->get_input_port(0).size() << std::endl;
  // VectorX<double> constant_vector =
  //     Eigen::MatrixXd::Zero(hamr->get_input_port(0).size(), 1);
  // auto input =
  // builder.AddSystem<drake::systems::ConstantVectorSource<double>>(
  //     constant_vector);

  // std::cout << "2" << std::endl;

  // // connections
  // // builder.Connect(hamr->get_output_port(0),
  // state_logger->get_input_port()); builder.Connect(input->get_output_port(),
  // hamr->get_input_port(0));
  // // builder.Connect(input->get_output_port(),
  // //                 hamr_w_act->get_input_port_voltage());
  // // builder.Connect(hamr_w_act->get_output_port_state(),
  // //                 state_logger->get_input_port());
  // // builder.Connect(hamr->get_output_port(0),
  // //                 visualizer->get_input_port(0));

  // auto sys = builder.Build();
  // Simulator<double> simulator(*sys);

  // // set initial state
  // hamr->set_state_vector(&simulator.get_mutable_context(),
  //                        hamr->HamrInitialState());
  // // auto hamr_context =
  // //     &sys->GetMutableSubsystemContext(*hamr,
  // //     &simulator.get_mutable_context());
  // // // hamr_context->get_mutable_continuous_state_vector().SetFromVector(
  // // //     hamr->HamrInitialState());
  // // hamr_context->get_mutable_discrete_state_vector().SetFromVector(
  // //     hamr->HamrInitialState());

  // std::cout << "3" << std::endl;
  // // std::cout << hamr_context->get_discrete_state_vector() << std::endl;

  // lcm.StartReceiveThread();
  // simulator.set_publish_every_time_step(true);
  // simulator.set_target_realtime_rate(1000);

  // std::cout << "4" << std::endl;
  // simulator.Initialize();
  // std::cout << "5" << std::endl;
  // simulator.StepTo(T);

  // // print to file
  // // std::ofstream state_log;
  // // state_log.open("/home/nddoshi/dev/RL-sandbox/hamr/dat_file.csv",
  // // std::ios::out);
  // // if(state_log.is_open())
  // // {
  // //   state_log<<state_logger->data();
  // //   state_log.close();
  // // }
  // // else std::cout << "unable to open file" << std::endl;
}

}  // namespace
}  // namespace hamr

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return hamr::DoMain();
}

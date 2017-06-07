#include "drake/systems/sensors/image_to_lcm_message.h"

#include <string>

#include "bot_core/images_t.hpp"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/systems/rendering/pose_stamped_t_pose_vector_translator.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include <gflags/gflags.h>

DEFINE_double(duration, 1, "Total duration of the simulation in secondes.");

namespace drake {

using multibody::joints::kQuaternion;

namespace systems {
namespace sensors {

int main(int argc, char* argv[]) {
  const std::string kSensorName = "Foo";
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();
  ::drake::lcm::DrakeLcm real_lcm;

  auto tree = std::make_unique<RigidBodyTree<double>>();
  std::string filename("/home/eacousineau/proj/tri/proj/dart_impl/_data/"
                       "kuniBottle/model-1_4.sdf");

  drake::parsers::sdf::AddModelInstancesFromSdfFileToWorld(
      filename, kQuaternion, tree.get());

  systems::DiagramBuilder<double> builder;

  auto plant = builder.AddSystem<RigidBodyPlant<double>>(move(tree));
  plant->set_name("rigid_body_plant");
  plant->set_normal_contact_parameters(3000, 10);
  plant->set_friction_contact_parameters(0.9, 0.5, 0.01);

  auto viz = builder.AddSystem<systems::DrakeVisualizer>(
      plant->get_rigid_body_tree(), &real_lcm);
  builder.Connect(plant->state_output_port(),
                  viz->get_input_port(0));

  const double dt = 0.01;
  auto rgbd_camera =
      builder.template AddSystem<RgbdCamera>(
          "rgbd_camera", plant->get_rigid_body_tree(),
          Eigen::Vector3d(-1., 0., 1.),
          Eigen::Vector3d(0., M_PI_4, 0.), M_PI_4, true, dt);
  rgbd_camera->set_name("rgbd_camera");

  auto image_to_lcm_message =
      builder.template AddSystem<ImageToLcmMessage>();
  image_to_lcm_message->set_name("converter");

  auto lcm_publisher = builder.template AddSystem(
      lcm::LcmPublisherSystem::Make<bot_core::images_t>(
          "DRAKE_RGB_IMAGE", &real_lcm));
  lcm_publisher->set_name("publisher");
  lcm_publisher->set_publish_period(dt);

  rendering::PoseStampedTPoseVectorTranslator translator("test_frame");
  auto pose_lcm_publisher = builder.template AddSystem<
    lcm::LcmPublisherSystem>("DRAKE_RGBD_CAMERA_POSE",
                             translator, &real_lcm);
  pose_lcm_publisher->set_name("pose_lcm_publisher");
  pose_lcm_publisher->set_publish_period(dt);

  builder.Connect(
      plant->get_output_port(0),
      rgbd_camera->state_input_port());

  builder.Connect(
      rgbd_camera->color_image_output_port(),
      image_to_lcm_message->color_image_input_port());

  builder.Connect(
      rgbd_camera->depth_image_output_port(),
      image_to_lcm_message->depth_image_input_port());

//  builder.Connect(
//      rgbd_camera->label_image_output_port(),
//      image_to_lcm_message->label_image_input_port());

  builder.Connect(
      image_to_lcm_message->images_t_msg_output_port(),
      lcm_publisher->get_input_port(0));

  builder.Connect(
      rgbd_camera->camera_base_pose_output_port(),
      pose_lcm_publisher->get_input_port(0));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto simulator = std::make_unique<systems::Simulator<double>>(
      *diagram, std::move(context));

  simulator->set_publish_at_initialization(true);
  simulator->set_publish_every_time_step(true);
  simulator->Initialize();
  simulator->StepTo(FLAGS_duration);

  return 0;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::systems::sensors::main(argc, argv);
}

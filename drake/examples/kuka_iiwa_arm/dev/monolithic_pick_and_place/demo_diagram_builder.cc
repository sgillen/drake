#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/demo_diagram_builder.h"

#include <memory>

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"


#include "bot_core/images_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/systems/sensors/image_to_lcm_message.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/rendering/pose_stamped_t_pose_vector_translator.h"

#include "drake/systems/sensors/depth_sensor.h"
#include "drake/systems/sensors/depth_sensor_to_lcm_point_cloud_message.h"
#include "bot_core/pointcloud_t.hpp"

namespace drake {

using systems::DrakeVisualizer;
using systems::DiagramBuilder;
using lcm::DrakeLcm;
using systems::RigidBodyPlant;

using Eigen::Vector3d;
using Eigen::Matrix3d;
using systems::sensors::RgbdCamera;
using systems::sensors::ImageToLcmMessage;
using systems::lcm::LcmPublisherSystem;
using systems::rendering::PoseStampedTPoseVectorTranslator;
using std::make_unique;

using systems::sensors::DepthSensor;
using systems::sensors::DepthSensorSpecification;
using systems::sensors::DepthSensorToLcmPointCloudMessage;

namespace examples {
using manipulation::schunk_wsg::SchunkWsgTrajectoryGenerator;
using manipulation::schunk_wsg::SchunkWsgStatusSender;

namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

template <typename T>
StateMachineAndPrimitives<T>::StateMachineAndPrimitives(
    const RigidBodyTree<T>& iiwa_tree, const double iiwa_action_primitive_rate,
    const double wsg_action_primitive_rate) {
  this->set_name("StateMachineAndPrimitives");
  DiagramBuilder<T> builder;

  iiwa_move_ = builder.template AddSystem<IiwaMove>(iiwa_tree,
                                                    iiwa_action_primitive_rate);
  iiwa_move_->set_name("IiwaMove");

  gripper_action_ =
      builder.template AddSystem<GripperAction>(wsg_action_primitive_rate);
  gripper_action_->set_name("GripperAction");

  // TODO(naveenoid): This needs to take robot base orientation into account
  // eventually.
  Isometry3<T> iiwa_base = Isometry3<T>::Identity();
  iiwa_base.translation() = kRobotBase;

  pick_and_place_state_machine_ =
      builder.template AddSystem<PickAndPlaceStateMachineSystem>(iiwa_base);
  pick_and_place_state_machine_->set_name("PickAndPlaceStateMachine");

  input_port_iiwa_robot_state_t_ = builder.ExportInput(
      pick_and_place_state_machine_->get_input_port_iiwa_state());

  input_port_box_robot_state_t_ = builder.ExportInput(
      pick_and_place_state_machine_->get_input_port_box_state());

  input_port_wsg_status_ = builder.ExportInput(
      pick_and_place_state_machine_->get_input_port_wsg_status());

  builder.Connect(pick_and_place_state_machine_->get_output_port_iiwa_action(),
                  iiwa_move_->get_primitive_input_port());
  builder.Connect(pick_and_place_state_machine_->get_output_port_wsg_action(),
                  gripper_action_->get_primitive_input_port());

  builder.Connect(
      iiwa_move_->get_status_output_port(),
      pick_and_place_state_machine_->get_input_port_iiwa_action_status());

  builder.Connect(
      gripper_action_->get_status_output_port(),
      pick_and_place_state_machine_->get_input_port_wsg_action_status());

  output_port_iiwa_command_ =
      builder.ExportOutput(iiwa_move_->get_robot_plan_output_port());
  output_port_wsg_command_ =
      builder.ExportOutput(gripper_action_->get_robot_plan_output_port());

  builder.BuildInto(this);
}
template class StateMachineAndPrimitives<double>;

/**
 * Convenience to infer type.
 */
template <typename T>
std::unique_ptr<T> CreateUnique(T* obj) {
  return std::unique_ptr<T>(obj);
}

template <typename T>
struct IiwaWsgPlantGeneratorsEstimatorsAndVisualizer<T>::Impl {
  // Generic serializer shared between two sensor types.
  PoseStampedTPoseVectorTranslator pose_translator_;

  RgbdCamera* rgbd_camera_{};
  ImageToLcmMessage* image_to_lcm_message_{};
  LcmPublisherSystem* image_lcm_pub_{};
  LcmPublisherSystem* rgbd_camera_pose_lcm_pub_{};

  DepthSensor* depth_sensor_{};
  LcmPublisherSystem* depth_sensor_pose_lcm_pub_{};

  void CreateAndConnectCamera(
      DiagramBuilder<double>* pbuilder,
      DrakeLcm* plcm,
      const IiwaAndWsgPlantWithStateEstimator<double>* pplant) {

    bool use_rgbd_camera = false;
    bool use_depth_sensor = true;

    const double pi = M_PI;

    const auto& rigid_body_tree = pplant->get_plant().get_rigid_body_tree();

    // Camera.
    /*
     * Obtained from director:
     * >>> c = view.camera()
     * >>> print(c.GetFocalPoint())
     * >>> print(c.GetPosition())
    */
//    const Vector3d position(0.05, -0.5, 0.7);
//    const Vector3d focal_point(3.75, 4.75, 3.25);
//    const auto orientation = CameraEulerAngle(position, focal_point);
    const double y_offset = -0.5;
    const double planar_distance = 2;
    const double height = 2;
    const Vector3d position(planar_distance, planar_distance + y_offset, height);
    const Vector3d orientation(0, 20, -135); // degrees

    if (use_rgbd_camera) {
      // Adapted from: .../image_to_lcm_message_demo.cc

      auto rgbd_camera_instance = new RgbdCamera(
          "rgbd_camera", rigid_body_tree,
          position, orientation * pi / 180, pi / 4, true);
      rgbd_camera_ = pbuilder->AddSystem(CreateUnique(rgbd_camera_instance));
      rgbd_camera_->set_name("rgbd_camera");

      // Connect directly to ground truth state.
      pbuilder->Connect(
          pplant->get_output_port_plant_state(),
          rgbd_camera_->state_input_port());

      // Image to LCM.
      image_to_lcm_message_ =
          pbuilder->template AddSystem<ImageToLcmMessage>();
      image_to_lcm_message_->set_name("converter");

      pbuilder->Connect(
          rgbd_camera_->color_image_output_port(),
          image_to_lcm_message_->color_image_input_port());

      pbuilder->Connect(
          rgbd_camera_->depth_image_output_port(),
          image_to_lcm_message_->depth_image_input_port());

      pbuilder->Connect(
          rgbd_camera_->label_image_output_port(),
          image_to_lcm_message_->label_image_input_port());

      // Camera image publisher.
      image_lcm_pub_ = pbuilder->template AddSystem(
          LcmPublisherSystem::Make<bot_core::images_t>(
              "DRAKE_RGB_IMAGE", plcm));
      image_lcm_pub_->set_name("publisher");
      image_lcm_pub_->set_publish_period(0.01);

      pbuilder->Connect(
          image_to_lcm_message_->images_t_msg_output_port(),
          image_lcm_pub_->get_input_port(0));

      // Camera pose publisher (to visualize)
      rgbd_camera_pose_lcm_pub_ = pbuilder->template AddSystem<
        LcmPublisherSystem>("DRAKE_RGBD_CAMERA_POSE",
                            pose_translator_, plcm);
      rgbd_camera_pose_lcm_pub_->set_name("pose_lcm_publisher");
      rgbd_camera_pose_lcm_pub_->set_publish_period(0.01);

      pbuilder->Connect(
          rgbd_camera_->camera_base_pose_output_port(),
          rgbd_camera_pose_lcm_pub_->get_input_port(0));
    }

    if (use_depth_sensor) {
      // Try out an equivalent depth sensor (or rather, just send out raycasts).
      DepthSensorSpecification specification;
  //    DepthSensorSpecification::set_octant_1_spec(&specification);
      auto* spec = &specification;
      spec->set_min_yaw(-pi / 4);
      spec->set_max_yaw(pi / 4);
      spec->set_min_pitch(-pi / 4);
      spec->set_max_pitch(pi / 4);
      spec->set_num_yaw_values(240);
      spec->set_num_pitch_values(320);
      spec->set_min_range(0);
      spec->set_max_range(100);

      auto world_body = const_cast<RigidBody<double>*>(&rigid_body_tree.world());
      RigidBodyFrame<double> frame("depth_sensor", world_body,
                                   position, orientation * pi / 180);
      auto depth_sensor_instance = new DepthSensor(
          "depth_sensor", rigid_body_tree, frame, specification);
      depth_sensor_ = pbuilder->AddSystem(CreateUnique(depth_sensor_instance));
      depth_sensor_->set_name("depth_sensor");

      // Connect directly to ground truth state.
      pbuilder->Connect(
          pplant->get_output_port_plant_state(),
          depth_sensor_->get_rigid_body_tree_state_input_port());

      // Point Cloud to LCM.
      // From: depth_sensor_to_lcm_point_cloud_message_demo
      auto depth_to_lcm_message_ =
          pbuilder->template AddSystem<DepthSensorToLcmPointCloudMessage>(specification);
      const std::string kSensorName = "DEPTH";
      auto lcm_publisher_depth_ = pbuilder->template AddSystem(
          LcmPublisherSystem::Make<bot_core::pointcloud_t>(
              "DRAKE_POINTCLOUD_" + kSensorName, plcm));
      pbuilder->Connect(
          depth_to_lcm_message_->pointcloud_message_output_port(),
          lcm_publisher_depth_->get_input_port(0));

      // Camera pose publisher (to visualize)
      depth_sensor_pose_lcm_pub_ = pbuilder->template AddSystem<
        LcmPublisherSystem>("DRAKE_DEPTH_SENSOR_POSE",
                            pose_translator_, plcm);
      depth_sensor_pose_lcm_pub_->set_name("pose_lcm_publisher");
      depth_sensor_pose_lcm_pub_->set_publish_period(0.01);

      pbuilder->Connect(
          depth_sensor_->get_pose_output_port(),
          depth_sensor_pose_lcm_pub_->get_input_port(0));
    }
  }
};

template <typename T>
IiwaWsgPlantGeneratorsEstimatorsAndVisualizer<T>::
    IiwaWsgPlantGeneratorsEstimatorsAndVisualizer(
        DrakeLcm* lcm, const int chosen_box, const double update_interval,
        Eigen::Vector3d box_position, Eigen::Vector3d box_orientation) {
  this->set_name("IiwaWsgPlantGeneratorsEstimatorsAndVisualizer");

  DiagramBuilder<T> builder;
  ModelInstanceInfo<double> iiwa_instance, wsg_instance, box_instance;

  std::unique_ptr<systems::RigidBodyPlant<double>> model_ptr =
      BuildCombinedPlant<double>(&iiwa_instance, &wsg_instance, &box_instance,
                                 chosen_box, box_position, box_orientation);
  plant_ = builder.template AddSystem<IiwaAndWsgPlantWithStateEstimator<T>>(
      std::move(model_ptr), iiwa_instance, wsg_instance, box_instance);
  plant_->set_name("plant");

  drake_visualizer_ = builder.template AddSystem<DrakeVisualizer>(
      plant_->get_plant().get_rigid_body_tree(), lcm);
  drake_visualizer_->set_name("drake_visualizer");

  builder.Connect(plant_->get_output_port_plant_state(),
                  drake_visualizer_->get_input_port(0));

  iiwa_trajectory_generator_ =
      builder.template AddSystem<RobotPlanInterpolator>(
          drake::GetDrakePath() + kIiwaUrdf, update_interval);
  iiwa_trajectory_generator_->set_name("iiwa_trajectory_generator");

  builder.Connect(plant_->get_output_port_iiwa_state(),
                  iiwa_trajectory_generator_->get_state_input_port());
  builder.Connect(
      iiwa_trajectory_generator_->get_state_output_port(),
      plant_->get_input_port_iiwa_state_command());
  builder.Connect(
      iiwa_trajectory_generator_->get_acceleration_output_port(),
      plant_->get_input_port_iiwa_acceleration_command());

  input_port_iiwa_plan_ =
      builder.ExportInput(iiwa_trajectory_generator_->get_plan_input_port());

  wsg_trajectory_generator_ =
      builder.template AddSystem<SchunkWsgTrajectoryGenerator>(
          plant_->get_output_port_wsg_state().size(), 0);
  wsg_trajectory_generator_->set_name("wsg_trajectory_generator");

  builder.Connect(plant_->get_output_port_wsg_state(),
                  wsg_trajectory_generator_->get_state_input_port());
  builder.Connect(wsg_trajectory_generator_->get_output_port(0),
                  plant_->get_input_port_wsg_command());
  input_port_wsg_plan_ =
      builder.ExportInput(wsg_trajectory_generator_->get_command_input_port());

  output_port_iiwa_robot_state_msg_ =
      builder.ExportOutput(plant_->get_output_port_iiwa_robot_state_msg());
  output_port_box_robot_state_msg_ =
      builder.ExportOutput(plant_->get_output_port_box_robot_state_msg());

  // Sets up a WSG Status sender.
  wsg_status_sender_ = builder.template AddSystem<SchunkWsgStatusSender>(
      plant_->get_output_port_wsg_state().size(), 0, 0);
  wsg_status_sender_->set_name("wsg_status_sender");

  builder.Connect(plant_->get_output_port_wsg_state(),
                  wsg_status_sender_->get_input_port(0));
  output_port_wsg_status_ =
      builder.ExportOutput(wsg_status_sender_->get_output_port(0));

  // Sets up a RGBD camera.
  impl_.reset(new Impl());
  impl_->CreateAndConnectCamera(&builder, lcm, plant_);

  builder.BuildInto(this);
}

// Define destructor since we need information for deleter in implementation
// pattern.
template<typename T>
IiwaWsgPlantGeneratorsEstimatorsAndVisualizer<T>::~IiwaWsgPlantGeneratorsEstimatorsAndVisualizer()
{}

template <typename T>
void IiwaWsgPlantGeneratorsEstimatorsAndVisualizer<T>::InitializeIiwaPlan(
    const VectorX<T>& q0, systems::Context<T>* context) const {
  auto plan_source_context =
      this->GetMutableSubsystemContext(
          context, iiwa_trajectory_generator_);
  iiwa_trajectory_generator_->Initialize(
      context->get_time(), q0,
      plan_source_context->get_mutable_state());
}


template class IiwaWsgPlantGeneratorsEstimatorsAndVisualizer<double>;

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

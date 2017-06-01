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
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/depth_sensor_output.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"

#include "drake/common/scoped_timer.h"

namespace drake {

using std::make_shared;
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

using std::unique_ptr;

namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

template <typename T_>
class LeafSystemMixin : public systems::LeafSystem<T_> {
 public:
  typedef T_ T;
  typedef systems::Context<T> Context;
  typedef systems::DiscreteValues<T> DiscreteValues;
  typedef systems::SystemOutput<T> SystemOutput;
  using Inport = systems::InputPortDescriptor<T>;
  using Outport = systems::OutputPortDescriptor<T>;
  // Blech.
  template <typename U>
  using Value = systems::Value<U>;
};

class WallClockPublisher : public LeafSystemMixin<double> {
 public:
  WallClockPublisher(double period_sec) {
    timer_.reset(new timing::Timer());
    prev_time_.reset(new double(0.));
    this->DeclarePeriodicDiscreteUpdate(period_sec, 0.);
  }

  typedef double T;

 protected:
  void DoCalcOutput(const Context&, SystemOutput*) const override {}
  void DoCalcDiscreteVariableUpdates(
      const Context& context, DiscreteValues* discrete_state) const override {
    unused(discrete_state);
    if (timer_->is_active()) {
      double elapsed = timer_->stop();
      double sim_elapsed = context.get_time() - *prev_time_;
      std::cout << "Elapsed time: " << elapsed << std::endl;
      std::cout << "  - Sim: " << sim_elapsed << std::endl;
    }
    timer_->start();
    *prev_time_ = context.get_time();
  }
 private:
  // HACK
  unique_ptr<double> prev_time_;
  unique_ptr<timing::Timer> timer_;
};

using systems::rendering::PoseVector;
using systems::sensors::DepthSensorOutput;
using systems::sensors::ImageDepth32F;
using systems::sensors::CameraInfo;

// HACK: Cribbed from rgbd_camera.cc
const int kImageWidth = 640;  // In pixels
const int kImageHeight = 480;  // In pixels

// // Sigh...
//template <typename T>
//auto MakeValue(T&& value) {
//  return systems::Value<T>(std::forward<T>(value));
//}

// HACK: Actually outputs DepthSensorOutput, with very non-descriptive sensor
// information.
// TODO(eric.cousineau): Decouple point-cloud definition from DepthSensorOutput,
// or decouple / use composition for LCM point cloud translation from this.
class DepthImageToPointCloud : public LeafSystemMixin<double> {
  // Model after: DpethSensorToLcmPointCloudMessage
 public:
  DepthImageToPointCloud(const CameraInfo& camera_info)
      : camera_info_(camera_info) {
    ImageDepth32F depth_image(kImageWidth, kImageHeight);
    depth_image_input_port_index_ = DeclareAbstractInputPort(
        Value<ImageDepth32F>(depth_image)).get_index();
    // TODO(eric.cousineau): Determine proper way to pass a basic point cloud.
    Eigen::Matrix3Xd point_cloud(3, depth_image.size());
    output_port_index_ =
        DeclareAbstractOutputPort(Value<Eigen::Matrix3Xd>(point_cloud)).get_index();
  }

 protected:
  void DoCalcOutput(const Context& context, SystemOutput* output) const override {
    const auto& depth_image =
        EvalAbstractInput(context, depth_image_input_port_index_)
        ->GetValue<ImageDepth32F>();
    auto& point_cloud = output->GetMutableData(output_port_index_)
                        ->GetMutableValue<Eigen::Matrix3Xd>();
    RgbdCamera::ConvertDepthImageToPointCloud(depth_image, camera_info_, &point_cloud);
  }

 private:
  DepthSensorSpecification spec_;
  const CameraInfo& camera_info_;
  int depth_image_input_port_index_{};
  int output_port_index_{};
};


// TODO(eric.cousineau): Replace with LCM Translator when PR lands
class PointCloudToLcmPointCloud : public LeafSystemMixin<double> {
 public:
  typedef Eigen::Matrix3Xd Data;
  typedef bot_core::pointcloud_t Message;

  PointCloudToLcmPointCloud() {
    input_port_index_ =
        DeclareAbstractInputPort(Value<Data>()).get_index();
    pose_input_port_index_ =
        DeclareVectorInputPort(PoseVector<double>()).get_index();
    output_port_index_ =
        DeclareAbstractOutputPort(Value<Message>()).get_index();
  }

  const Inport& get_inport() const {
    return get_input_port(input_port_index_);
  }
  const Inport& get_pose_inport() const {
    return get_input_port(pose_input_port_index_);
  }
  const Outport& get_outport() const {
    return get_output_port(output_port_index_);
  }
 protected:
  void DoCalcOutput(
      const Context& context, SystemOutput* output) const {
    const Data& point_cloud = EvalAbstractInput(context, input_port_index_)
                              ->GetValue<Data>();
    Message& message = output->GetMutableData(output_port_index_)
                        ->GetMutableValue<Message>();
    auto pose_WS = EvalVectorInput<PoseVector>(context,
                                               pose_input_port_index_);
    auto X_WS = pose_WS->get_isometry();
    // Stolen from DepthSensorToLcmPointCloudMessage
    message.frame_id = std::string(RigidBodyTreeConstants::kWorldName);
    message.n_points = point_cloud.cols();
    message.points.clear();
    for (int i = 0; i < point_cloud.cols(); ++i) {
      const auto& point_S = point_cloud.col(i);
      Eigen::Vector3f point_W = (X_WS * point_S).cast<float>();
      message.points.push_back({point_W(0), point_W(1), point_W(2)});
    }
    message.n_channels = 0;
  }
 private:
  int input_port_index_{};
  int pose_input_port_index_{};
  int output_port_index_{};
};

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
  PoseStampedTPoseVectorTranslator pose_translator_{"camera"};

  RgbdCamera* rgbd_camera_{};
  ImageToLcmMessage* image_to_lcm_message_{};
  LcmPublisherSystem* image_lcm_pub_{};
  LcmPublisherSystem* rgbd_camera_pose_lcm_pub_{};

  DepthSensor* depth_sensor_{};
  LcmPublisherSystem* depth_sensor_pose_lcm_pub_{};

  void CreateAndConnectCamera(
      DiagramBuilder<double>* pbuilder,
      DrakeLcm* plcm,
      IiwaAndWsgPlantWithStateEstimator<double>* pplant) {

    bool use_rgbd_camera = true;
    bool use_depth_sensor = false;

    const double pi = M_PI;

    // HACK
    auto& rigid_body_tree = const_cast<RigidBodyTree<double>&>(
        pplant->get_plant().get_rigid_body_tree());

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

    pbuilder->template AddSystem<WallClockPublisher>(0.001);

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

      // Publish depth image.
      auto depth_to_pc = pbuilder->template AddSystem<DepthImageToPointCloud>(
            rgbd_camera_->depth_camera_info());
      pbuilder->Connect(
            rgbd_camera_->depth_image_output_port(),
            depth_to_pc->get_input_port(0));
      typedef PointCloudToLcmPointCloud Converter;
      auto pc_to_lcm = pbuilder->template AddSystem<Converter>();
      pbuilder->Connect(
            depth_to_pc->get_output_port(0),
            pc_to_lcm->get_inport());
      pbuilder->Connect(
            rgbd_camera_->camera_base_pose_output_port(),
            pc_to_lcm->get_pose_inport());
      // Add LCM publisher
      auto depth_lcm_pub = pbuilder->template AddSystem<LcmPublisherSystem>(
          LcmPublisherSystem::Make<Converter::Message>("DRAKE_RGBD_POINT_CLOUD",
                                                       plcm));
      pbuilder->Connect(
            pc_to_lcm->get_outport(),
            depth_lcm_pub->get_input_port(0));
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
      spec->set_num_yaw_values(60);
      spec->set_num_pitch_values(80);
      spec->set_min_range(0);
      spec->set_max_range(5);

      auto world_body = &rigid_body_tree.world();
      auto frame = make_shared<RigidBodyFrame<double>>(
          "depth_sensor", world_body, position, orientation * pi / 180);
      rigid_body_tree.addFrame(frame);
      auto depth_sensor_instance = new DepthSensor(
          "depth_sensor", rigid_body_tree, *frame, specification);
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
        depth_sensor_->get_sensor_state_output_port(),
        depth_to_lcm_message_->depth_readings_input_port());
      pbuilder->Connect(
          depth_to_lcm_message_->pointcloud_message_output_port(),
          lcm_publisher_depth_->get_input_port(0));
      pbuilder->Connect(
          depth_sensor_->get_pose_output_port(),
          depth_to_lcm_message_->pose_input_port());

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
  const bool use_slow_meshes = false;

  std::unique_ptr<systems::RigidBodyPlant<double>> model_ptr =
      BuildCombinedPlant<double>(&iiwa_instance, &wsg_instance, &box_instance,
                                 chosen_box, box_position, box_orientation,
                                 use_slow_meshes);
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

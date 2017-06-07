#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/perception_hack.h"

#include <memory>

#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

#include "drake/common/drake_path.h"

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
#include "drake/systems/sensors/image.h"

#include "drake/common/scoped_timer.h"

#include "drake/manipulation/estimators/dev/articulated_state_estimator.h"
#include "drake/common/find_resource.h"

#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"

#include "drake/manipulation/estimators/dev/tree_state_portion.h"
#include "drake/common/drake_optional.h"

namespace drake {


using std::unique_ptr;
using std::string;
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

using manipulation::ArticulatedStateEstimator;
using manipulation::LeafSystemMixin;
using manipulation::GetHierarchicalPositionNameList;

namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

class WallClockPublisher : public LeafSystemMixin<double> {
 public:
  WallClockPublisher() {
    // HACK: Store previous time in state, if possible.
    timer_.reset(new timing::Timer());
    prev_time_.reset(new double(0.));
    this->DeclarePerStepAction(
          systems::DiscreteEvent<double>::kDiscreteUpdateAction);
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
  DepthImageToPointCloud(const CameraInfo& camera_info,
                         bool use_depth_frame = true)
      : camera_info_(camera_info),
        ues_depth_frame_(use_depth_frame) {
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
    RgbdCamera::ConvertDepthImageToPointCloud(depth_image, camera_info_,
                                              &point_cloud);
    if (ues_depth_frame_) {
      // TODO(eric.cousineau): Change system to use Depth sensor pose output
      // port, once the PR lands for this, to get the proper frame externally.
      Eigen::Matrix3d R_BD;
      R_BD <<
          0, 0, 1,
          -1, 0, 0,
          0, -1, 0;
      // Project from `D` to `B`
      point_cloud = (R_BD * point_cloud).eval();
    }
    drake::log()->info("Convert to depth cloud: {}", context.get_time());
  }

 private:
  DepthSensorSpecification spec_;
  const CameraInfo& camera_info_;
  bool ues_depth_frame_{};
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
    int n_points = point_cloud.cols();
    auto X_WS = pose_WS->get_isometry();
    // Stolen from DepthSensorToLcmPointCloudMessage
    message.frame_id = std::string(RigidBodyTreeConstants::kWorldName);
    message.n_points = n_points;
    message.points.clear();
    for (int i = 0; i < n_points; ++i) {
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

//using manipulation::VectorPortion;
//using manipulation::VectorSlice;
//using Eigen::VectorXd;

//class VectorSliceTranslator : public LeafSystemMixin<double> {
// public:
//  typedef VectorSlice<double> Slice;

//  VectorSliceTranslator(const Slice& input_slice)
//    : input_slice_(input_slice) {
//    DeclareInputPort(systems::kVectorValued, input_slice_.super_size());
//    DeclareOutputPort(systems::kVectorValued, input_slice_.size());
//  }

//  void DoCalcOutput(
//      const Context& context, SystemOutput* output) const override {
//    VectorXd input = EvalVectorInput(context, 0)->CopyToVector();
//    auto&& subset = output->GetMutableVectorData(0)->get_mutable_value();
//    input_slice_.ReadFromSuperset(input, subset);
//  }
// private:
//  Slice input_slice_;
//};

/**
 * Convenience to infer type.
 */
template <typename T>
std::unique_ptr<T> CreateUnique(T* obj) {
  return std::unique_ptr<T>(obj);
}

struct PerceptionHack::Impl {
  // Generic serializer shared between two sensor types.
  PoseStampedTPoseVectorTranslator pose_translator_{"camera"};

  RgbdCamera* rgbd_camera_{};
  ImageToLcmMessage* image_to_lcm_message_{};
  LcmPublisherSystem* image_lcm_pub_{};
  LcmPublisherSystem* rgbd_camera_pose_lcm_pub_{};

  DepthSensor* depth_sensor_{};
  LcmPublisherSystem* depth_sensor_pose_lcm_pub_{};

  void CreateAndConnectCamera(
      DiagramBuilder* pbuilder,
      DrakeLcm* plcm,
      TreePlant* pplant,
      const ReverseIdMap& plant_id_map) {

    drake::log()->set_level(spdlog::level::trace);

    bool use_rgbd_camera = true;
    bool use_depth_sensor = false;

    const double pi = M_PI;

    // HACK
    auto& rigid_body_tree = const_cast<RigidBodyTree<double>&>(
        pplant->get_plant().get_rigid_body_tree());

    // Camera.
    const Vector3d position(0, 2, 2);
    const Vector3d orientation(0, 20, -90); // degrees

    pbuilder->template AddSystem<WallClockPublisher>();

    const double camera_dt = 0.033; // ~30 Hz
    if (use_rgbd_camera) {
      bool use_estimator = false;

      // Adapted from: .../image_to_lcm_message_demo.cc

      auto rgbd_camera_instance = new RgbdCamera(
          "rgbd_camera", rigid_body_tree,
          position, orientation * pi / 180, pi / 4, true,
          camera_dt);
      rgbd_camera_ = pbuilder->AddSystem(CreateUnique(rgbd_camera_instance));
      rgbd_camera_->set_name("rgbd_camera");

      // Connect directly to ground truth state.
      pbuilder->Connect(
          pplant->get_output_port_plant_state(),
          rgbd_camera_->state_input_port());

//      using namespace systems::sensors;
//      auto rgbd_zoh =
//          new AbstractZOHDiagram<ImageRgba8U, ImageDepth32F, ImageLabel16I>(0.03);
//      pbuilder->AddSystem(CreateUnique(rgbd_zoh));
//      for (int i = 0; i < 3; ++i) {
//        pbuilder->Connect(rgbd_camera_->get_output_port(i),
//                          rgbd_zoh->get_input_port(i));
//      }
//      auto&& color_image_output_port = rgbd_zoh->get_output_port(0);
//      auto&& depth_image_output_port = rgbd_zoh->get_output_port(1);
////      auto&& label_image_output_port = rgbd_zoh->get_output_port(2);

      auto&& color_image_output_port = rgbd_camera_->get_output_port(0);
      auto&& depth_image_output_port = rgbd_camera_->get_output_port(1);
      auto&& camera_base_pose_output_port = rgbd_camera_->camera_base_pose_output_port();

      bool do_publish = true;
      if (do_publish) {
        // Image to LCM.
        image_to_lcm_message_ =
            pbuilder->template AddSystem<ImageToLcmMessage>();
        image_to_lcm_message_->set_name("converter");

        pbuilder->Connect(
            color_image_output_port,
            image_to_lcm_message_->color_image_input_port());

        pbuilder->Connect(
            depth_image_output_port,
            image_to_lcm_message_->depth_image_input_port());

        // This port has been disabled.
//        pbuilder->Connect(
//            label_image_output_port,
//            image_to_lcm_message_->label_image_input_port());

        // Camera image publisher.
        image_lcm_pub_ = pbuilder->template AddSystem(
            LcmPublisherSystem::Make<bot_core::images_t>(
                "DRAKE_IMAGE_RGBD", plcm));
        image_lcm_pub_->set_name("publisher");
        image_lcm_pub_->set_publish_period(camera_dt);

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
            camera_base_pose_output_port,
            rgbd_camera_pose_lcm_pub_->get_input_port(0));
      }

      // Publish depth image.
      auto depth_to_pc = pbuilder->template AddSystem<DepthImageToPointCloud>(
            rgbd_camera_->depth_camera_info());
      pbuilder->Connect(
            depth_image_output_port,
            depth_to_pc->get_input_port(0));

      if (do_publish) {
        typedef PointCloudToLcmPointCloud Converter;
        auto pc_to_lcm = pbuilder->template AddSystem<Converter>();
        pbuilder->Connect(
              depth_to_pc->get_output_port(0),
              pc_to_lcm->get_inport());
        pbuilder->Connect(
              camera_base_pose_output_port,
              pc_to_lcm->get_pose_inport());
        // Add LCM publisher
        auto depth_lcm_pub = pbuilder->template AddSystem<LcmPublisherSystem>(
            LcmPublisherSystem::Make<Converter::Message>("DRAKE_POINTCLOUD_RGBD",
                                                         plcm));
        depth_lcm_pub->set_name("depth_point_cloud_lcm_publisher");
        depth_lcm_pub->set_publish_period(camera_dt);
        pbuilder->Connect(
              pc_to_lcm->get_outport(),
              depth_lcm_pub->get_input_port(0));
      }

      if (use_estimator) {
        string base_path = "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/";
        string config_file =
            drake::FindResource(base_path + "dart_config/iiwa_test.yaml").get_absolute_path_or_throw();

        bool add_velocities = true;
        auto input_position_names = GetHierarchicalPositionNameList(
                                      pplant->get_plant().get_rigid_body_tree(),
                                      plant_id_map, add_velocities);
        std::cout << "names\n";
        for (auto&& name : input_position_names) {
          std::cout << "  " << name << "\n";
        }

        auto estimator = pbuilder->template AddSystem<ArticulatedStateEstimator>(
                           config_file, &rgbd_camera_->depth_camera_info(),
                           input_position_names);

        pbuilder->Connect(depth_to_pc->get_output_port(0),
                          estimator->inport_point_cloud());
        pbuilder->Connect(depth_image_output_port,
                          estimator->inport_depth_image());

        pbuilder->Connect(pplant->get_output_port_plant_state(),
                          estimator->inport_tree_q_measurement());
//        pbuilder->Connect(estimator->outport_tree_state_estimate(),
//                          ???);
      }
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
      auto lcm_publisher_depth_ = pbuilder->template AddSystem(
          LcmPublisherSystem::Make<bot_core::pointcloud_t>(
              "DRAKE_POINTCLOUD_DEPTH", plcm));
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


void PerceptionHack::Inject(DiagramBuilder* pbuilder, DrakeLcm* plcm,
                            TreePlant* pplant, const ReverseIdMap& plant_id_map) {
  impl_.reset(new Impl());
  impl_->CreateAndConnectCamera(pbuilder, plcm, pplant, plant_id_map);
}

PerceptionHack::~PerceptionHack() {}

/*
  const bool use_slow_meshes = false;

  // Sets up a RGBD camera.
  impl_.reset(new Impl());
  impl_->CreateAndConnectCamera(&builder, lcm, plant_);
*/

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

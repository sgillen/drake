#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/perception_hack.h"

#include <memory>

#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

#include "drake/common/drake_path.h"

//#include "bot_core/images_t.hpp"
#include "robotlocomotion/image_array_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
//#include "drake/systems/sensors/image_to_lcm_message.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/rendering/pose_stamped_t_pose_vector_translator.h"

#include <bot_lcmgl_client/lcmgl.h>

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

#include "drake/manipulation/estimators/dev/abstract_zoh.h"
#include "drake/manipulation/estimators/dev/vector_slice.h"
#include "drake/common/drake_optional.h"

#include "drake/common/call_matlab.h"
#include "drake/systems/primitives/constant_vector_source.h"

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
using systems::sensors::ImageToLcmImageArrayT;
using systems::lcm::LcmPublisherSystem;
using systems::rendering::PoseStampedTPoseVectorTranslator;
using std::make_unique;

using systems::sensors::DepthSensor;
using systems::sensors::DepthSensorSpecification;
using systems::sensors::DepthSensorToLcmPointCloudMessage;

using manipulation::ArticulatedStateEstimator;
using manipulation::LeafSystemMixin;
using manipulation::GetHierarchicalPositionNameList;

using systems::AbstractZOH;

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

typedef Eigen::Matrix3Xd PointCloud;

// HACK: Cribbed from rgbd_camera.cc
const int kImageWidth = 640;  // In pixels
const int kImageHeight = 480;  // In pixels

// // Sigh...
//template <typename T>
//auto MakeValue(T&& value) {
//  return systems::Value<T>(std::forward<T>(value));
//}

/**
 * Transform two poses in the order they are supplied.
 *
 * Conceptually:
 *   inputs: X_AB, X_BC
 *   output: X_AC
 */
class PoseTransformer : public LeafSystemMixin<double> {
 public:
  PoseTransformer() {
    DeclareVectorInputPort(PoseVector<double>());
    DeclareVectorInputPort(PoseVector<double>());
    DeclareVectorOutputPort(PoseVector<double>());
  }
 protected:
  void DoCalcOutput(const Context& context, SystemOutput* output) const override {
    auto&& pose_a =
        EvalVectorInput<PoseVector>(context, 0);
    auto&& pose_b =
        EvalVectorInput<PoseVector>(context, 1);
    auto&& pose_out =
        *dynamic_cast<PoseVector<double>*>(output->GetMutableVectorData(0));
    Eigen::Isometry3d X_out =
        pose_a->get_isometry() * pose_b->get_isometry();
    pose_out.set_translation(Eigen::Translation3d(X_out.translation()));
    pose_out.set_rotation(Eigen::Quaterniond(X_out.rotation()));
  }
};

// HACK: Actually outputs DepthSensorOutput, with very non-descriptive sensor
// information.
// Provides point cloud in depth sensor frame.
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
    PointCloud point_cloud(3, depth_image.size());
    output_port_index_ =
        DeclareAbstractOutputPort(Value<PointCloud>(point_cloud)).get_index();
  }
  const Inport& get_depth_image_inport() const {
    return get_input_port(depth_image_input_port_index_);
  }
 protected:
  void DoCalcOutput(const Context& context, SystemOutput* output) const override {
    const auto& depth_image =
        EvalAbstractInput(context, depth_image_input_port_index_)
        ->GetValue<ImageDepth32F>();
    auto& point_cloud = output->GetMutableData(output_port_index_)
                        ->GetMutableValue<PointCloud>();
    RgbdCamera::ConvertDepthImageToPointCloud(depth_image, camera_info_,
                                              &point_cloud);
    drake::log()->info("Convert to depth cloud: {}", context.get_time());
    manipulation::PrintValidPoints(point_cloud, "Converter");
  }
 private:
  DepthSensorSpecification spec_;
  const CameraInfo& camera_info_;
  bool ues_depth_frame_{};
  int depth_image_input_port_index_{};
  int output_port_index_{};
};

class PointCloudVisualizer::Impl {
 public:
  struct Log {
    std::vector<double> times;
    std::vector<Matrix3Xd> data;
    void AddData(double t, const Matrix3Xd& in) {
      times.push_back(t);
      data.push_back(in);
    }
    const Matrix3Xd& GetData(double t) const {
      DRAKE_DEMAND(times.size() > 0);
      int i = 0;
      while (i + 1 < (int)times.size() && t > times[i]) {
        ++i;
      }
      // Will latch last time, 'cause HACK.
      return data[i];
    }
  };
  mutable Log log_;
  mutable bot_lcmgl_t* lcmgl_{};
};

PointCloudVisualizer::PointCloudVisualizer(drake::lcm::DrakeLcm* lcm, double dt) {
  impl_.reset(new Impl());
  Impl& impl = *impl_;
  impl.lcmgl_ = bot_lcmgl_init(lcm->get_lcm_instance()->getUnderlyingLCM(),
                          "point_cloud_playback");
  DeclareAbstractInputPort();
  DeclareVectorInputPort(PoseVector<double>()).get_index();
  DeclarePublishPeriodSec(dt);
}

PointCloudVisualizer::~PointCloudVisualizer() {}

void PointCloudVisualizer::PlaybackFrame(double t) const {
  Impl& impl = *impl_;
  const Matrix3Xd& data = impl.log_.GetData(t);
  PublishCloud(data);
}

void PointCloudVisualizer::PublishCloud(const Eigen::Matrix3Xd& cloud) const {
  Impl& impl = *impl_;
  auto* lcmgl_ = impl.lcmgl_;
  bot_lcmgl_begin(lcmgl_, LCMGL_POINTS);
  double width = 640;
  double height = 480;
  int downsample = 5;
  for (int v = 0; v < height; v += downsample) {
    for (int u = 0; u < width; u += downsample) {
      // TODO(eric.cousineau): Use random downsampling from actual estimation.
      int dv = rand() % downsample;
      int du = rand() % downsample;
      int i = (v + dv) * width + (u + du);
      auto&& pt = cloud.col(i);
      if (!std::isnan(pt[0]) && !std::isinf(pt[0])) {
        bot_lcmgl_color3f(lcmgl_, 0.5, 1.0, 0.5);
        bot_lcmgl_vertex3f(lcmgl_, pt[0], pt[1], pt[2]);
      }
    }
  }
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_switch_buffer(lcmgl_);
}

using systems::Context;
using systems::SystemOutput;

void PointCloudVisualizer::DoPublish(const Context<double>& context) const {
  const Matrix3Xd& point_cloud = EvalAbstractInput(context, 0)
                                 ->GetValue<Matrix3Xd>();
  auto pose_WS = EvalVectorInput<PoseVector>(context, 1);
  auto X_WS = pose_WS->get_isometry();
  auto point_cloud_W = X_WS * point_cloud;
  impl_->log_.AddData(context.get_time(), point_cloud_W);
  PublishCloud(point_cloud_W);
}

void PointCloudVisualizer::DoCalcOutput(const Context<double>&,
                                        SystemOutput<double>*) const {}

// Publishes point cloud in world frame.
// TODO(eric.cousineau): Replace with LCM Translator when PR lands
class PointCloudToLcmPointCloud : public LeafSystemMixin<double> {
 public:
  typedef Eigen::Matrix3Xd Data;
  typedef bot_core::pointcloud_t Message;

  PointCloudToLcmPointCloud() {
    input_port_index_ =
        DeclareAbstractInputPort(Value<Data>()).get_index();
    output_port_index_ =
        DeclareAbstractOutputPort(Value<Message>()).get_index();
    pose_input_port_index_ =
        DeclareVectorInputPort(PoseVector<double>()).get_index();
  }

  const Inport& get_pose_inport() const {
    return get_input_port(pose_input_port_index_);
  }
  const Inport& get_inport() const {
    return get_input_port(input_port_index_);
  }
  const Outport& get_outport() const {
    return get_output_port(output_port_index_);
  }
 protected:
  void DoCalcOutput(
      const Context& context, SystemOutput* output) const {
    const Data& point_cloud = EvalAbstractInput(context, input_port_index_)
                              ->GetValue<Data>();
    auto pose_WS = EvalVectorInput<PoseVector>(context,
                                               pose_input_port_index_);
    auto X_WS = pose_WS->get_isometry();
    const Data& point_cloud_world = X_WS * point_cloud;
    Message& message = output->GetMutableData(output_port_index_)
                        ->GetMutableValue<Message>();
    int n_points = point_cloud_world.cols();
    // Stolen from DepthSensorToLcmPointCloudMessage
    message.frame_id = std::string(RigidBodyTreeConstants::kWorldName);
    message.n_points = n_points;
    message.points.clear();
    for (int i = 0; i < n_points; ++i) {
      const auto& point_S = point_cloud_world.col(i);
      Eigen::Vector3f point_W = point_S.cast<float>();
      message.points.push_back({point_W(0), point_W(1), point_W(2)});
    }
    message.n_channels = 0;
  }
 private:
  int input_port_index_{};
  int output_port_index_{};
  int pose_input_port_index_{};
};

//template <typename Type, typename Port>
//void HoldOutport(const Port** pptr, double dt, const Type& value = Type) {

//}

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
  ImageToLcmImageArrayT* image_to_lcm_message_{};
  LcmPublisherSystem* image_lcm_pub_{};
  LcmPublisherSystem* rgbd_camera_pose_lcm_pub_{};

  DepthSensor* depth_sensor_{};
  LcmPublisherSystem* depth_sensor_pose_lcm_pub_{};
  DrakeVisualizer* estimator_vis_{};

  PointCloudVisualizer* pc_vis_{};

  void CreateAndConnectCamera(
      DiagramBuilder* pbuilder,
      DrakeLcm* plcm,
      TreePlant* pplant,
      const ReverseIdMap& plant_id_map) {

//    drake::log()->set_level(spdlog::level::trace);

    bool use_rgbd_camera = true;
    bool use_depth_sensor = false;
    bool use_wall_clock_pub = false;

    const double pi = M_PI;

    // HACK
    auto& rigid_body_tree = const_cast<RigidBodyTree<double>&>(
        pplant->get_plant().get_rigid_body_tree());

    // Camera.
    const Vector3d position(0, 2, 2);
    const Vector3d orientation(0, 20, -90); // degrees

    if (use_wall_clock_pub) {
      pbuilder->template AddSystem<WallClockPublisher>();
    }

    const double camera_dt = 0.033; // ~30 Hz
    if (use_rgbd_camera) {
      bool use_estimator = true;

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

      auto&& color_image_output_port = rgbd_camera_->get_output_port(0);
      auto&& depth_image_output_port = rgbd_camera_->get_output_port(1);
      auto&& camera_base_pose_output_port = rgbd_camera_->camera_base_pose_output_port();

      // Project from `D` (depth frame) to `B` (camera frame), per documentation
      // for RgbdCamera.
      // The camera presently outputs X_WB, but we want X_WD.
      // TODO(eric.cousineau): Change system to use Depth sensor pose output
      // port, once the PR lands for this, to get the proper frame externally.
      Eigen::Matrix3d R_BD;
      R_BD <<
          0, 0, 1,
          -1, 0, 0,
          0, -1, 0;
      Eigen::Vector3d p_BD(0, 0.02, 0);
      Eigen::Isometry3d X_BD;
      X_BD.linear() = R_BD;
      X_BD.translation() = p_BD;
      // TODO(eric.cousineau): This was very inconvenient. Is there a simpler
      // way to do this, possibly just in service of double templates?
      PoseVector<double> pose_BD; // sigh...
      pose_BD.set_rotation(Eigen::Quaterniond(X_BD.rotation()));
      pose_BD.set_translation(Eigen::Translation3d(X_BD.translation()));
      using systems::ConstantVectorSource;
      auto* depth_to_camera_pose =
          pbuilder->template AddSystem<ConstantVectorSource<double>>(
              pose_BD);
      auto camera_pose_transformer =
          pbuilder->template AddSystem<PoseTransformer>();

      pbuilder->Connect(camera_base_pose_output_port,
                        camera_pose_transformer->get_input_port(0));
      pbuilder->Connect(depth_to_camera_pose->get_output_port(),
                        camera_pose_transformer->get_input_port(1));
      auto&& depth_camera_pose_output_port =
          camera_pose_transformer->get_output_port(0);

      // Camera pose publisher (to visualize)
      rgbd_camera_pose_lcm_pub_ = pbuilder->template AddSystem<
        LcmPublisherSystem>("DRAKE_RGBD_CAMERA_POSE",
                            pose_translator_, plcm);
      rgbd_camera_pose_lcm_pub_->set_name("pose_lcm_publisher");
      rgbd_camera_pose_lcm_pub_->set_publish_period(0.01);
      pbuilder->Connect(
          camera_base_pose_output_port,
          rgbd_camera_pose_lcm_pub_->get_input_port(0));

      bool do_publish = false;
      if (do_publish) {
        // Image to LCM.
        image_to_lcm_message_ =
            pbuilder->template AddSystem<ImageToLcmImageArrayT>(
                "color", "depth", "label");
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
            LcmPublisherSystem::Make<robotlocomotion::image_array_t>(
                "DRAKE_RGBD_CAMERA_IMAGES", plcm));
        image_lcm_pub_->set_name("publisher");
        image_lcm_pub_->set_publish_period(camera_dt);

        pbuilder->Connect(
            image_to_lcm_message_->image_array_t_msg_output_port(),
            image_lcm_pub_->get_input_port(0));
      }

      // Convert depth image.
      auto depth_to_pc = pbuilder->template AddSystem<DepthImageToPointCloud>(
            rgbd_camera_->depth_camera_info());
      pbuilder->Connect(
            depth_image_output_port,
            depth_to_pc->get_depth_image_inport());

      auto* pc_zoh = pbuilder->template AddSystem<AbstractZOH<PointCloud>>(camera_dt);
      pbuilder->Connect(
            depth_to_pc->get_output_port(0),
            pc_zoh->get_input_port(0));
      auto&& pc_output_port = pc_zoh->get_output_port(0);

      pc_vis_ =
          pbuilder->template AddSystem<PointCloudVisualizer>(plcm, camera_dt);
      pbuilder->Connect(
            depth_to_pc->get_output_port(0),
            pc_vis_->get_input_port(0));
      pbuilder->Connect(
            depth_camera_pose_output_port,
            pc_vis_->get_input_port(1));

      if (do_publish) {
        typedef PointCloudToLcmPointCloud Converter;
        auto pc_to_lcm = pbuilder->template AddSystem<Converter>();
        pbuilder->Connect(
              pc_output_port,
              pc_to_lcm->get_inport());
        pbuilder->Connect(
              depth_camera_pose_output_port,
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

        ArticulatedStateEstimator* estimator =
            pbuilder->template AddSystem<ArticulatedStateEstimator>(
                config_file, &rgbd_camera_->depth_camera_info(),
                input_position_names, camera_dt);

        pbuilder->Connect(pc_output_port,
                          estimator->inport_point_cloud());
        pbuilder->Connect(depth_image_output_port,
                          estimator->inport_depth_image());
        pbuilder->Connect(depth_camera_pose_output_port,
                          estimator->inport_depth_camera_pose());

        pbuilder->Connect(pplant->get_output_port_plant_state(),
                          estimator->inport_tree_q_measurement());

        // Create visualizer with prefix
        estimator_vis_ =
            pbuilder->template AddSystem<DrakeVisualizer>(
                estimator->get_tree(),
                plcm,
                true,
                "ESTIMATOR_");
        estimator_vis_->set_name("estimator_visualizer");

        pbuilder->Connect(estimator->outport_tree_state_estimate(),
                          estimator_vis_->get_input_port(0));

//        // Make sure it works with duplicating the original plant.
//        auto estimator_vis =
//            pbuilder->template AddSystem<DrakeVisualizer>(
//              rigid_body_tree,
//              plcm,
//              false,
//              "ESTIMATOR_");
//        estimator_vis->set_name("estimator_visualizer");
//        pbuilder->Connect(pplant->get_output_port_plant_state(),
//                          estimator_vis->get_input_port(0));
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

systems::DrakeVisualizer*PerceptionHack::GetEstimationVisualizer()
{
  return impl_->estimator_vis_;
}

PointCloudVisualizer* PerceptionHack::GetPointCloudVisualizer()
{
  return impl_->pc_vis_;
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

#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/perception_hack.h"

#include <memory>
#include <bot_lcmgl_client/lcmgl.h>

#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

#include "robotlocomotion/image_array_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/rendering/pose_stamped_t_pose_vector_translator.h"

#include "drake/systems/primitives/constant_vector_source.h"

#include "bot_core/pointcloud_t.hpp"
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/systems/sensors/image.h"

#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"

#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/abstract_zoh.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/dart_util.h"

namespace drake {

using std::unique_ptr;
using std::string;
using std::make_shared;
using systems::DiagramBuilder;
using lcm::DrakeLcm;
using systems::RigidBodyPlant;

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Matrix3Xd;
using systems::sensors::RgbdCamera;
using systems::sensors::ImageToLcmImageArrayT;
using systems::lcm::LcmPublisherSystem;
using systems::rendering::PoseStampedTPoseVectorTranslator;
using std::make_unique;

using systems::rendering::PoseVector;
using systems::sensors::ImageDepth32F;
using systems::sensors::CameraInfo;
using systems::AbstractZOH;

typedef Eigen::Matrix3Xd PointCloud;

// HACK: Cribbed from rgbd_camera.cc
const int kImageWidth = 640;  // In pixels
const int kImageHeight = 480;  // In pixels

namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

typedef double T;

/**
 * Simple mixin to get simplified aliases.
 */
template <typename T_>
class LeafSystemMixin : public systems::LeafSystem<T_> {
 public:
  typedef T_ T;
  typedef systems::Context<T> Context;
  typedef systems::DiscreteValues<T> DiscreteValues;
  typedef systems::SystemOutput<T> SystemOutput;
  typedef systems::State<T> State;
  using Inport = systems::InputPortDescriptor<T>;
  using Outport = systems::OutputPort<T>;
  template <typename U>
  using Value = systems::Value<U>;
};

/**
 * Transform two poses in the order they are supplied.
 *
 * Conceptually:
 *   inputs: X_AB, X_BC
 *   output: X_AC
 */
class PoseTransformer : public LeafSystemMixin<T> {
 public:
  PoseTransformer() {
    DeclareVectorInputPort(PoseVector<T>());
    DeclareVectorInputPort(PoseVector<T>());
    DeclareVectorOutputPort(PoseVector<T>(),
                            &PoseTransformer::CalcPose);
  }
 protected:
  void CalcPose(const Context& context,
                    PoseVector<T>* ppose_out) const {
    auto&& pose_a =
        EvalVectorInput<PoseVector>(context, 0);
    auto&& pose_b =
        EvalVectorInput<PoseVector>(context, 1);
    Eigen::Isometry3d X_out =
        pose_a->get_isometry() * pose_b->get_isometry();
    auto& pose_out = *ppose_out;
    pose_out.set_translation(Eigen::Translation3d(X_out.translation()));
    pose_out.set_rotation(Eigen::Quaterniond(X_out.rotation()));
  }
};

class DepthImageNoise : public LeafSystemMixin<T> {
 public:
  DepthImageNoise(double noise_rel_magnitude)
    : noise_rel_magnitude_(noise_rel_magnitude) {
    ImageDepth32F depth_image(kImageWidth, kImageHeight);
    DeclareAbstractInputPort(Value<ImageDepth32F>(depth_image));
    DeclareAbstractOutputPort(depth_image,
                              &DepthImageNoise::CalcNoise);
  }
 protected:
  void CalcNoise(const Context& context,
                    ImageDepth32F* pout) const {
    const auto& in =
        EvalAbstractInput(context, 0)->GetValue<ImageDepth32F>();
    ImageDepth32F& out = *pout;

    for (int v = 0; v < in.height(); v++) {
      for (int u = 0; u < in.width(); u++) {
        double scale = 1 + noise_rel_magnitude_ *
                       state_.noise_distribution(state_.noise_generator);
        *out.at(u, v) = scale * *in.at(u, v);
      }
    }
  }
 private:
  double noise_rel_magnitude_{};
  // Modelling after: RandomSource
  // TODO: Use Abstract State.
  using Generator = std::mt19937;
  using Distribution = std::normal_distribution<T>;
  struct RandomState {
    Generator noise_generator;
    Distribution noise_distribution;
  };
  mutable RandomState state_;
};

/**
 * Draw a camera frustrum view leveraging a point cloud straight from a depth
 * image. This will iterate along the borders, and place a point at a maximum
 * distance if it was not found in the point cloud.
 */
class CameraFrustrumVisualizer : public LeafSystemMixin<T> {
 public:
  CameraFrustrumVisualizer(DrakeLcm* lcm,
      const CameraInfo& camera_info,
      double period_sec, double min_depth, double max_depth)
      : camera_info_(camera_info),
        min_depth_(min_depth),
        max_depth_(max_depth) {
    DeclarePublishPeriodSec(period_sec);
    DeclareAbstractInputPort();
    DeclareVectorInputPort(PoseVector<T>());
    lcmgl_ = bot_lcmgl_init(lcm->get_lcm_instance()->getUnderlyingLCM(),
                                 "camera_frustrum");
  }

 protected:
  struct Entry {
    Eigen::Isometry3d X_WC;
    Matrix3Xd corners_near;
    Matrix3Xd corners_far;
    Matrix3Xd points_W;

    void Draw(bot_lcmgl_t* lcmgl_, double min_depth) const {
      // Draw each point connected to the next (looping).
      bot_lcmgl_begin(lcmgl_, LCMGL_LINES);
      bot_lcmgl_color3f(lcmgl_, 0.5, 1.0, 0.5);
      int nn = points_W.cols();
      for (int i = 0; i < nn; ++i) {
        auto ci = points_W.col(i);
        int n = (i + 1) % nn; // Wrap for drawing box
        auto cn = points_W.col(n);
        bot_lcmgl_vertex3f(lcmgl_, ci[0], ci[1], ci[2]);
        bot_lcmgl_vertex3f(lcmgl_, cn[0], cn[1], cn[2]);
      }
      // Draw from each of the four corners.
      for (int i = 0; i < 4; ++i) {
        auto cc = X_WC.translation();
        auto c0 = corners_near.col(i);
        int next = (i + 1) % 4;
        auto c0n = corners_near.col(next);
        auto ci = corners_far.col(i);
        // Camera origin to near clipping plane.
//        bot_lcmgl_color3f(lcmgl_, 0.8, 1.0, 0.8);
        bot_lcmgl_vertex3f(lcmgl_, cc[0], cc[1], cc[2]);
        bot_lcmgl_vertex3f(lcmgl_, c0[0], c0[1], c0[2]);
        // Near to far clipping pane.
//        bot_lcmgl_color3f(lcmgl_, 0.5, 1.0, 0.5);
        bot_lcmgl_vertex3f(lcmgl_, c0[0], c0[1], c0[2]);
        bot_lcmgl_vertex3f(lcmgl_, ci[0], ci[1], ci[2]);
        // Near clipping pane box.
        bot_lcmgl_vertex3f(lcmgl_, c0[0], c0[1], c0[2]);
        bot_lcmgl_vertex3f(lcmgl_, c0n[0], c0n[1], c0n[2]);
      }
      bot_lcmgl_end(lcmgl_);
      bot_lcmgl_switch_buffer(lcmgl_);
    }
  };

  void DoPublish(const Context& context) const override {
    const Matrix3Xd& point_cloud_C = EvalAbstractInput(context, 0)
                                   ->GetValue<Matrix3Xd>();
    auto&& pose =
        EvalVectorInput<PoseVector>(context, 1);
    if (point_cloud_C.cols() == 0) {
      drake::log()->info("Skipping empty point cloud");
      return;
    }
    Eigen::Isometry3d X_WC = pose->get_isometry();
    // TODO(eric.cousineau): Generalize ConvertDepthImageToPointCloud to accept
    // coordinates?
    const int w = camera_info_.width();
    const int h = camera_info_.height();
    // Copied:
    const float cx = camera_info_.center_x();
    const float cy = camera_info_.center_y();
    const float fx_inv = 1.f / camera_info_.focal_x();
    const float fy_inv = 1.f / camera_info_.focal_y();

    int npix = 5;
    DRAKE_DEMAND(point_cloud_C.cols() == w * h);
    DRAKE_DEMAND(w % npix == 0);
    DRAKE_DEMAND(h % npix == 0);
    int nw = w / npix;
    int nh = h / npix;
    int nn = 2 * (nw + nh);

    Matrix3Xd points_W(3, nn);
    int index = 0;
    std::vector<int> corner_indices;  // Laziness.
    auto add_pt = [&](int u, int v) {
      int i = v * w + u;
      ASSERT_THROW_FMT(index < nn, "{} !< {}", index, nn);
      ASSERT_THROW_FMT(i < point_cloud_C.cols(),
                       "{} !< {}", i, point_cloud_C.cols());
      Vector3d pt_C = point_cloud_C.col(i);
      if (std::isnan(pt_C[0]) || std::isinf(pt_C[0])) {
        // Recompute with extended depth.
        double z = max_depth_;
        pt_C(0) = z * (u - cx) * fx_inv;
        pt_C(1) = z * (v - cy) * fy_inv;
        pt_C(2) = z;
      } else if (pt_C[2] > max_depth_) {
        // Ensure that point's length is saturated.
        pt_C *= max_depth_ / pt_C[2];
      }
      points_W.col(index++) = X_WC * pt_C;
    };
    // Make two strips of points in order, including edges.
    // Make iteration clockwise in camera frame.
    corner_indices.push_back(index);
    for (int u = 0; u < w; u += npix) {
      add_pt(u, 0);
    }
    corner_indices.push_back(index);
    for (int v = 0; v < h; v += npix) {
      add_pt(w - 1, v);
    }
    corner_indices.push_back(index);
    for (int u = w - 1; u >= 0; u -= npix) {
      add_pt(u, h - 1);
    }
    corner_indices.push_back(index);
    for (int v = h - 1; v >= 0; v -= npix) {
      add_pt(0, v);
    }
    DRAKE_DEMAND(index == nn);
    // Process corners to create view box for minimum depth.
    Matrix3Xd corners_close(3, 4);
    Matrix3Xd corners_far(3, 4);
    int i = 0;
    for (int index : corner_indices) {
      // Cheap cheating.
      Vector3d pt_W = points_W.col(index);
      Vector3d pt_C = X_WC.inverse() * pt_W;
      Vector3d pt_near_C = pt_C * (min_depth_ / pt_C[2]);
      corners_far.col(i) = pt_W;
      corners_close.col(i) = X_WC * pt_near_C;
      i++;
    }
    Entry entry {X_WC, corners_close, corners_far, points_W};
//    log_.AddData(context.get_time(), entry);
    entry.Draw(lcmgl_, min_depth_);
  }
 private:
  bot_lcmgl_t* lcmgl_{};
  const CameraInfo& camera_info_;
  double min_depth_{};
  double max_depth_{};
//  mutable SimpleLog<Entry> log_;
};

class DepthImageToPointCloud : public LeafSystemMixin<T> {
  // Model after: DepthSensorToLcmPointCloudMessage
 public:
  DepthImageToPointCloud(const CameraInfo& camera_info)
      : camera_info_(camera_info) {
    ImageDepth32F depth_image(kImageWidth, kImageHeight);
    depth_image_input_port_index_ = DeclareAbstractInputPort(
        Value<ImageDepth32F>(depth_image)).get_index();
    // TODO(eric.cousineau): Determine proper way to pass a basic point cloud.
    PointCloud point_cloud(3, depth_image.size());
    output_port_index_ =
        DeclareAbstractOutputPort(
            point_cloud,
            &DepthImageToPointCloud::CalcPointCloud).get_index();
  }
  const Inport& get_depth_image_inport() const {
    return get_input_port(depth_image_input_port_index_);
  }
 protected:
  void CalcPointCloud(const Context& context,
                      PointCloud* ppoint_cloud) const {
    const auto& depth_image =
        EvalAbstractInput(context, depth_image_input_port_index_)
        ->GetValue<ImageDepth32F>();
    auto& point_cloud = *ppoint_cloud;
    Eigen::Matrix3Xf point_cloud_f;
    RgbdCamera::ConvertDepthImageToPointCloud(depth_image, camera_info_,
                                              &point_cloud_f);
    point_cloud = point_cloud_f.cast<double>();
    drake::log()->info("Convert to depth cloud: {}", context.get_time());
    manipulation::PrintValidPoints(point_cloud, "Converter");
  }
 private:
  const CameraInfo& camera_info_;
  int depth_image_input_port_index_{};
  int output_port_index_{};
};

// Publishes point cloud in world frame.
// TODO(eric.cousineau): Replace with LCM Translator when PR lands
class PointCloudToLcmPointCloud : public LeafSystemMixin<T> {
 public:
  typedef Eigen::Matrix3Xd Data;
  typedef bot_core::pointcloud_t Message;

  PointCloudToLcmPointCloud(int downsample)
      : downsample_(downsample) {
    input_port_index_ =
        DeclareAbstractInputPort(Value<Data>()).get_index();
    pose_input_port_index_ =
        DeclareVectorInputPort(PoseVector<T>()).get_index();
    output_port_index_ =
        DeclareAbstractOutputPort(
            Message(),
            &PointCloudToLcmPointCloud::CalcMessage).get_index();
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
  void CalcMessage(
      const Context& context, Message* pmessage) const {
    const Data& point_cloud = EvalAbstractInput(context, input_port_index_)
                              ->GetValue<Data>();
    const double width = kImageWidth;
    const double height = kImageHeight;
    Message& message = *pmessage;
    message.points.clear();
    message.frame_id = std::string(RigidBodyTreeConstants::kWorldName);
    message.n_channels = 0;
    message.n_points = 0;
    if (point_cloud.size() == 0) {
      drake::log()->warn("{}: Empty point cloud. Skipping", this->get_name());
      return;
    }
    ASSERT_THROW_FMT(point_cloud.cols() == width * height,
                     "{} != {} * {}", point_cloud.cols(), width, height);
    auto pose_WS = EvalVectorInput<PoseVector>(context,
                                               pose_input_port_index_);
    auto X_WS = pose_WS->get_isometry();
    const Data& point_cloud_W = X_WS * point_cloud;
    for (int v = 0; v < height; v += downsample_) {
      for (int u = 0; u < width; u += downsample_) {
        // TODO(eric.cousineau): Use random downsampling from actual estimation.
        int dv = 0; //rand() % downsample;
        int du = 0; //rand() % downsample;
        int i = (v + dv) * width + (u + du);
        Eigen::Vector3f point_W = point_cloud_W.col(i).cast<float>();
        message.points.push_back({point_W(0), point_W(1), point_W(2)});
      }
    }
    message.n_points = message.points.size();
  }
 private:
  int input_port_index_{};
  int output_port_index_{};
  int pose_input_port_index_{};
  int downsample_{};
};

/**
 * Convenience to infer type.
 */
template <typename T>
std::unique_ptr<T> CreateUnique(T* obj) {
  return std::unique_ptr<T>(obj);
}

class PerceptionHack::Impl {
 public:
  // Generic serializer shared between two sensor types.
  PoseStampedTPoseVectorTranslator pose_translator_{"camera"};

  RgbdCamera* rgbd_camera_{};
  ImageToLcmImageArrayT* image_to_lcm_message_{};
  LcmPublisherSystem* image_lcm_pub_{};
  LcmPublisherSystem* rgbd_camera_pose_lcm_pub_{};

  CameraFrustrumVisualizer* cf_vis_{};

  void CreateAndConnectCamera(
      DiagramBuilder* pbuilder,
      DrakeLcm* plcm,
      TreePlant* pplant) {

    // bool use_wall_clock_pub = false;

    const double pi = M_PI;

    // HACK
    auto& rigid_body_tree = const_cast<RigidBodyTree<T>&>(
        pplant->get_plant().get_rigid_body_tree());

    // Camera.
    const Vector3d position(0, 2, 2);
    const Vector3d orientation(0, 20, -90); // degrees

    // if (use_wall_clock_pub) {
    //   pbuilder->template AddSystem<WallClockPublisher>();
    // }

    const double camera_dt = 1. / 30; // ~30 Hz
    if (true) {
      auto rgbd_camera_instance = new RgbdCamera(
          "rgbd_camera", rigid_body_tree,
          position, orientation * pi / 180, pi / 4, camera_dt, true); //,
          //camera_dt);
      rgbd_camera_ = pbuilder->AddSystem(CreateUnique(rgbd_camera_instance));
      rgbd_camera_->set_name("rgbd_camera");

      // Connect directly to ground truth state.
      pbuilder->Connect(
          pplant->get_output_port_plant_state(),
          rgbd_camera_->state_input_port());

      auto&& color_image_output_port = rgbd_camera_->get_output_port(0);

      auto depth_zoh =
          pbuilder->template AddSystem<AbstractZOH<ImageDepth32F>>(camera_dt);
      pbuilder->Connect(
            rgbd_camera_->get_output_port(1),
            depth_zoh->get_input_port(0));

      // Add noise.
      // 1% noise at 1 sigma (when sigma = 1)
      auto depth_noise =
          pbuilder->template AddSystem<DepthImageNoise>(0.01);
      pbuilder->Connect(
            depth_zoh->get_output_port(0),
            depth_noise->get_input_port(0));
      // TODO(eric.cousineau): Add ZOH on Depth Image noise (to keep
      // consistent).

      auto&& depth_image_output_port = depth_noise->get_output_port(0);
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
      PoseVector<T> pose_BD; // sigh...
      pose_BD.set_rotation(Eigen::Quaterniond(X_BD.rotation()));
      pose_BD.set_translation(Eigen::Translation3d(X_BD.translation()));
      using systems::ConstantVectorSource;
      auto* depth_to_camera_pose =
          pbuilder->template AddSystem<ConstantVectorSource<T>>(
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

      {
        // Convert depth image.
        auto pure_depth_to_pc = pbuilder->template AddSystem<DepthImageToPointCloud>(
              rgbd_camera_->depth_camera_info());
        pbuilder->Connect(
              depth_zoh->get_output_port(0),
              pure_depth_to_pc->get_depth_image_inport());

        const double min_depth = 0.5;
        const double max_depth = 5;
        cf_vis_ = pbuilder->template AddSystem<CameraFrustrumVisualizer>(
              plcm, rgbd_camera_->depth_camera_info(), camera_dt,
                    min_depth, max_depth);
        pbuilder->Connect(
              pure_depth_to_pc->get_output_port(0),
              cf_vis_->get_input_port(0));
        pbuilder->Connect(
              depth_camera_pose_output_port,
              cf_vis_->get_input_port(1));
      }

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

      if (true) { //do_publish) {
        typedef PointCloudToLcmPointCloud Converter;
        auto pc_to_lcm = pbuilder->template AddSystem<Converter>(5);
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
    }
  }
};


void PerceptionHack::Inject(DiagramBuilder* pbuilder, DrakeLcm* plcm,
                            TreePlant* pplant) {
  impl_.reset(new Impl());
  impl_->CreateAndConnectCamera(pbuilder, plcm, pplant);
}

PerceptionHack::~PerceptionHack() {}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

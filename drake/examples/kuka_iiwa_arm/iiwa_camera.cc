#include "drake/examples/kuka_iiwa_arm/iiwa_camera.h"

#include "drake/multibody/rigid_body_plant/frame_visualizer.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

Eigen::Isometry3d IiwaCameraFrames::X_GX() {
  // Attach Xtion (X) to wsg's end effector (G).
  Eigen::Isometry3d X_GX;
  // Based on tri_exp:27a7d76:book_pulling.cc (Jiaji's code), but changed to
  // align with present model frames.
  X_GX.linear() <<
      1, 0, 0,
      0, 0, 1,
      0, -1, 0;
  X_GX.translation() << 0, -0.015, -0.025;
  return X_GX;
}

Eigen::Matrix3d IiwaCameraFrames::R_WbX() {
  Eigen::Matrix3d R_WbX;
  R_WbX <<
      0, 0, 1,
      1, 0, 0,
      0, 1, 0;
  return R_WbX;
}

Eigen::Isometry3d IiwaCameraFrames::X_XB() {
  // Add sensor frame, `B` for RgbdCamera.
  Eigen::Isometry3d X_XB;
  X_XB.setIdentity();
  X_XB.linear() <<
      0, 1, 0,
      0, 0, 1,
      1, 0, 0;
  // TODO(eric.cousineau): Once relative transforms between B, C, D can be
  // defined, impose correct constraints on the frame offsets.
  X_XB.translation() << 0.0, 0.0325, 0.021;
  return X_XB;
}

bool IiwaCamera::is_movable() const {
  return &fixture_frame_->get_rigid_body() == &tree_->world();
}

IiwaCamera::IiwaCamera(TreeBuilder* tree_builder,
                       RigidBody<double> *parent,
                       const Eigen::Isometry3d& X_PX) {
  tree_builder->StoreModel(
        "xtion",
        "drake/examples/kuka_iiwa_arm/models/cameras/asus_xtion.urdf");

  fixture_frame_ =
      std::make_shared<RigidBodyFrame<double>>(
          name_ + "_fixture",
          parent, X_PX);

  tree_builder->mutable_tree().addFrame(fixture_frame_);
  tree_builder->AddModelInstanceToFrame(
      "xtion", fixture_frame_,
      drake::multibody::joints::kFixed);

  // Add sensor frame, `B` for RgbdCamera.
  Eigen::Isometry3d X_XB = IiwaCameraFrames::X_XB();
  // Compose frame to get proper orientation for RgbdCamera.
  sensor_frame_ =
      std::make_shared<RigidBodyFrame<double>>(
          name_ + "_sensor",
          fixture_frame_->get_mutable_rigid_body(),
          fixture_frame_->get_transform_to_body() * X_XB);
  tree_builder->mutable_tree().addFrame(sensor_frame_);
}

void IiwaCamera::AddToDiagram(systems::DiagramBuilder<double> *pbuilder,
                              const systems::RigidBodyPlant<double>& plant,
                              const lcm::DrakeLcm* lcm,
                              bool do_lcm_publish,
                              bool add_frame_visualizer) {
  auto& builder = *pbuilder;

  DRAKE_DEMAND(&plant.get_rigid_body_tree() == tree_);

  using namespace systems;
  using namespace systems::lcm;
  using namespace systems::sensors;

  RgbdCamera* rgbd_camera =
      builder.AddSystem<RgbdCamera>(
          name_, *tree_,
          *sensor_frame_, M_PI_4, true);

  builder.Connect(
      plant.get_output_port(0),
      rgbd_camera->state_input_port());

  // 30 Hz
  const double kDt = 1. / 30;
  const int kWidth = 640, kHeight = 480;

  Value<ImageRgba8U> image_rgb(kWidth, kHeight);
  auto zoh_rgb =
      builder.template AddSystem<ZeroOrderHold>(kDt, image_rgb);
  builder.Connect(rgbd_camera->color_image_output_port(),
                  zoh_rgb->get_input_port());

  Value<ImageDepth32F> image_depth(kWidth, kHeight);
  auto zoh_depth =
      builder.template AddSystem<ZeroOrderHold>(kDt, image_depth);
  builder.Connect(rgbd_camera->depth_image_output_port(),
                  zoh_depth->get_input_port());

  Value<ImageLabel16I> image_label(kWidth, kHeight);
  auto zoh_label =
      builder.template AddSystem<ZeroOrderHold>(kDt, image_label);
  builder.Connect(rgbd_camera->label_image_output_port(),
                  zoh_label->get_input_port());

  if (do_lcm_publish) {
    // Image to LCM.
    auto image_to_lcm_message =
        builder.AddSystem<ImageToLcmImageArrayT>(
            "color", "depth", "label");
    image_to_lcm_message->set_name("converter");

    builder.Connect(
        zoh_rgb->get_output_port(),
        image_to_lcm_message->color_image_input_port());

    builder.Connect(
        zoh_depth->get_output_port(),
        image_to_lcm_message->depth_image_input_port());

    builder.Connect(
        zoh_label->get_output_port(),
        image_to_lcm_message->label_image_input_port());

    // Camera image publisher.
    auto image_lcm_pub = builder.AddSystem(
        LcmPublisherSystem::Make<robotlocomotion::image_array_t>(
            lcm_channel_, lcm));
    image_lcm_pub->set_name("image_publisher");
    image_lcm_pub->set_publish_period(kDt);

    builder.Connect(
        image_to_lcm_message->image_array_t_msg_output_port(),
        image_lcm_pub->get_input_port(0));
  }

  if (add_frame_visualizer) {
    // Add frame visualizer.
    std::vector<RigidBodyFrame<double>> frames;
    for (const auto& frame : tree_->frames)
      frames.push_back(*frame);
    auto frame_viz =
        builder.AddSystem<FrameVisualizer>(
            tree_, frames, &lcm);
    builder.Connect(
        plant.get_output_port(0),
        frame_viz->get_input_port(0));
  }
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

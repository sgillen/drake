#pragma once

#include "drake/common/eigen_types.h"

#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram_builder.h"

#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/lcm/drake_lcm.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/**
 * Defines frames for the Xtion mounted on the WSG gripper on the IIWA.
 *
 * Frames:
 *  B - RgbdCamera frame
 *  C - RgbdCamera color frame
 *  D - RgbdCamera depth frame
 *  X - Xtion frame (X-left, Y-up, Z-forward)
 *     To visualize base pose, run `meshlab asus_xtion.obj`, go to
 *     Render > Show Axis.
 *  G - WSG Gripper frame.
 *  Wb - Base-world frame (X-forward, Y-left, Z-up).
 */
// TODO(eric.cousineau): Encode these in an URDF somewhere, where they are
// easily accessible?
class IiwaCameraFrames {
 public:
  /** Transform from Xtion to Gripper */
  static Eigen::Isometry3d X_GX();

  /** Orientation of Xtion in the base-world frame */
  static Eigen::Matrix3d R_WbX();

  /** Pose of RgbdCamera frame w.r.t. Xtion frame. */
  static Eigen::Isometry3d X_XB();
};

/**
 * Attaches an Asus Xtion PRO camera to a given frame.
 * Convenience provided for the IIWA + WSG setup.
 *
 * Example:
 * @code
 *  // Add to tree.
 *  IiwaCamera iiwa_camera(tree_builder, wsg_id);
 *  // Add to diagram.
 *  iiwa_camera.AddToDiagram(&builder, plant, &lcm);
 * @endcode
 */
class IiwaCamera {
 public:
  typedef manipulation::util::WorldSimTreeBuilder<double> TreeBuilder;

  /// Attach camera to a given body, with a given transform.
  IiwaCamera(TreeBuilder* tree_builder,
             RigidBody<double>* parent,
             const Eigen::Isometry3d& X_PX);

  /// Convenience for WSG.
  IiwaCamera(TreeBuilder* tree_builder,
             int wsg_id = -1)
    : IiwaCamera(tree_builder,
                 tree_builder->mutable_tree().FindBody("body", "", wsg_id),
                 IiwaCameraFrames::X_GX()) {}

  // Convenience for world-frame.
  IiwaCamera(TreeBuilder* tree_builder,
             const Eigen::Isometry3d& X_WX)
    : IiwaCamera(tree_builder,
                 &tree_builder->mutable_tree().world(),
                 X_WX) {}

  void AddToDiagram(systems::DiagramBuilder<double>* builder,
                    const systems::RigidBodyPlant<double>& plant,
                    const lcm::DrakeLcm* lcm,
                    bool do_lcm_publish = true,
                    bool add_frame_visualizer = true);

  bool is_movable() const;
 private:
  std::string name_{"xtion"};
  std::string lcm_channel_{"DRAKE_RGBD_CAMERA_IMAGES"};

  const RigidBodyTree<double>* tree_{};
  std::shared_ptr<RigidBodyFrame<double>> fixture_frame_;
  std::shared_ptr<RigidBodyFrame<double>> sensor_frame_;
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

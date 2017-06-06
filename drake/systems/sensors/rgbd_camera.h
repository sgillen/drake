#pragma once

#include <limits>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {
/// The RgbdCamera provides both RGB and depth images. Its image resolution is
/// fixed at VGA (640 x 480 pixels) for both the RGB and depth measurements. The
/// depth sensing range is 0.5 m to 5.0 m.
///
/// Let `W` be the world coordinate system. In addition to `W`, there are three
/// more coordinate systems that are associated with an RgbdCamera. They are
/// defined as follows:
///
///   * `B` - The camera's base coordinate system: X-forward, Y-left, and Z-up.
///
///   * `C` - the camera's color sensor's optical coordinate system: `X-right`,
///           `Y-down` and `Z-forward`.
///
///   * `D` - the camera's depth sensor's optical coordinate system: `X-right`,
///           `Y-down` and `Z-forward`.
///
/// The origins of `C` and `D` (i.e., `Co` and `Do`, respectively) are offset
/// from `B`'s origin (`Bo`) by 0 m in `B`'s X-axis, +0.02 m in `B`'s Y-axis,
/// and 0 m in `B`'s Z axis.  Since `C` and `D` are coincident, the depth image
/// is a "registered depth image" for the RGB image. No disparity between the
/// RGB and depth images are modeled. For more details about the poses of `C`
/// and `D`, see the class documentation of CameraInfo.
///
/// Output image format:
///   - The RGB image has four channels in the following order: red, green
///     blue, alpha. Each channel is represented by a uint8_t.
///
///   - The depth image has a depth channel represented by a float. The value
///     stored in the depth channel holds *the Z value in `D`.*  Note that this
///     is different from the range data used by laser range finders (like that
///     provided by DepthSensor) in which the depth value represents the
///     distance from the sensor origin to the object's surface.
///
///   - The label image has single channel represented by a int16_t. The value
///     stored in the channel holds a model ID which corresponds to an object
///     in the scene. For the pixels corresponding to no body, namely the sky
///     and the flat terrain, we assign Label::kNoBody and Label::kFlatTerrain,
///     respectively.
///
/// @ingroup sensor_systems
// TODO(kunimatsu-tri) Add support for the image publish capability.
class RgbdCamera : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdCamera)

  /// Converts a depth image obtained from RgbdCamera to a point cloud.  If a
  /// pixel in the depth image has NaN depth value, all the `(x, y, z)` values
  /// in the converted point will be NaN.
  /// Similarly, if a pixel has either InvalidDepth::kTooFar or
  /// InvalidDepth::kTooClose, the converted point will be
  /// InvalidDepth::kTooFar.
  /// Note that this matches the convention used by the Point Cloud Library
  /// (PCL).
  ///
  /// @param[in] depth_image The input depth image obtained from RgbdCamera.
  ///
  /// @param[in] camera_info The input camera info which is used for conversion.
  ///
  /// @param[out] point_cloud The pointer of output point cloud.
  static void ConvertDepthImageToPointCloud(const ImageDepth32F& depth_image,
                                            const CameraInfo& camera_info,
                                            Eigen::Matrix3Xd* point_cloud);

  /// Set of constants used to represent invalid depth values.
  /// Note that if a depth is not measurable, NaN will be set.
  class InvalidDepth {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InvalidDepth)
    /// The depth value when the max sensing range is exceeded.
    static constexpr float kTooFar{std::numeric_limits<float>::infinity()};

    /// The depth value when the min sensing range is violated because the
    /// object being sensed is too close. Note that this
    /// <a href="http://www.ros.org/reps/rep-0117.html">differs from ROS</a>,
    /// which uses negative infinity in this scenario. Drake uses zero because
    /// it results in less devastating bugs when users fail to check for the
    /// lower limit being hit and using negative infinity does not prevent users
    /// from writing bad code.
    static constexpr float kTooClose{0.f};
  };

  /// Set of labels used for label image.
  class Label {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Label)
    /// The label used for pixels correspond to nothing.
    static constexpr int16_t kNoBody{std::numeric_limits<int16_t>::max()};
    /// The label used for pixels correspond to the flat terrain.
    static constexpr int16_t kFlatTerrain{
      std::numeric_limits<int16_t>::max() - 1};
  };

  /// A constructor for %RgbdCamera that defines `B` using Euler angles.
  /// The pose of %RgbdCamera will be fixed to the world coordinate system
  /// throughout the simulation.
  ///
  /// @param name The name of the RgbdCamera.  This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  ///
  /// @param tree The RigidBodyTree containing the geometric description of the
  /// world. The life span of this parameter must exceed that of this class's
  /// instance. The maximum number of bodies in the `tree` must be less than
  /// 1536 based on the number of the colors used for the label image.
  ///
  /// @param position The x-y-z position of `B` in `W`. This defines the
  /// translation component of `X_WB`.
  ///
  /// @param orientation The roll-pitch-yaw orientation of `B` in `W`. This
  /// defines the orientation component of `X_WB`.
  ///
  /// @param fov_y The RgbdCamera's vertical field of view.
  ///
  /// @param show_window A flag for showing a visible window.  If this is false,
  /// offscreen rendering is executed. This is useful for debugging purposes.
  RgbdCamera(const std::string& name,
             const RigidBodyTree<double>& tree,
             const Eigen::Vector3d& position,
             const Eigen::Vector3d& orientation,
             double fov_y,
             bool show_window);

  /// A constructor for %RgbdCamera that defines `B` using a RigidBodyFrame.
  /// The pose of %RgbdCamera is fixed to a user-defined frame and will be
  /// updated during the simulation.
  ///
  /// @param name The name of the RgbdCamera.  This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  ///
  /// @param tree The RigidBodyTree containing the geometric description of the
  /// world. The life span of this parameter must exceed that of this class's
  /// instance. The maximum number of bodies in the `tree` must be less than
  /// 1536 based on the number of the colors used for the label image.
  ///
  /// @param frame The frame in @tree to which this camera is attached.
  ///
  /// @param fov_y The RgbdCamera's vertical field of view.
  ///
  /// @param show_window A flag for showing a visible window.  If this is false,
  /// offscreen rendering is executed. This is useful for debugging purposes.
  RgbdCamera(const std::string& name,
             const RigidBodyTree<double>& tree,
             const RigidBodyFrame<double>& frame,
             double fov_y,
             bool show_window);

  ~RgbdCamera();

  /// Reterns the color sensor's info.
  const CameraInfo& color_camera_info() const;

  /// Reterns the depth sensor's info.
  const CameraInfo& depth_camera_info() const;

  /// Returns `X_BC`.
  const Eigen::Isometry3d& color_camera_optical_pose() const;

  /// Returns `X_BD`.
  const Eigen::Isometry3d& depth_camera_optical_pose() const;

  /// Returns the RigidBodyFrame to which this RgbdCamera is attached.
  const RigidBodyFrame<double>& frame() const;

  /// Returns the RigidBodyTree to which this RgbdCamera is attached.
  const RigidBodyTree<double>& tree() const;

  /// Returns a descriptor of the vector valued input port that takes a vector
  /// of `q, v` corresponding to the positions and velocities associated with
  /// the RigidBodyTree.
  const InputPortDescriptor<double>& state_input_port() const;

  /// Returns a descriptor of the abstract valued output port that contains an
  /// RGBA image of the type ImageRgba8U.
  const OutputPortDescriptor<double>& color_image_output_port() const;

  /// Returns a descriptor of the abstract valued output port that contains an
  /// ImageDepth32F.
  const OutputPortDescriptor<double>& depth_image_output_port() const;

  /// Returns a descriptor of the abstract valued output port that contains an
  /// label image of the type ImageLabel16I.
  const OutputPortDescriptor<double>& label_image_output_port() const;

  /// Returns a descriptor of the vector valued output port that contains an
  /// PoseVector.
  const OutputPortDescriptor<double>& camera_base_pose_output_port() const;

 protected:
  /// Updates all the model frames for the renderer and outputs the rendered
  /// images.
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

 private:
  void Init(const std::string& name);

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake

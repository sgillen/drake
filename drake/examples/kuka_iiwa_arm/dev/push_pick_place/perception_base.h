#pragma once

#include "drake/common/eigen_types.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/camera_info.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace push_and_pick {

class PerceptionBase {
 public:
  PerceptionBase(systems::sensors::CameraInfo camera_info)
      : camera_info_(camera_info) {}

  virtual void Update(
    double time,
    const systems::sensors::ImageDepth32F& depth_image,
    const Eigen::Isometry3d& X_WD) = 0;

  virtual Eigen::Isometry3d EstimatePose() = 0;

  const CameraInfo& camera_info() const {
    return camera_info_;
  }

 private:
  const systems::sensors::CameraInfo camera_info_;
};

}  // namespace push_and_pick
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

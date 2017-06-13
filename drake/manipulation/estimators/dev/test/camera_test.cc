#include "drake/common/scoped_timer.h"
#include "drake/common/drake_path.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/manipulation/estimators/dev/dart_util.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/multibody/rigid_body_tree_construction.h"

using namespace drake;
using namespace drake::systems::sensors;
using namespace std;
using namespace Eigen;

typedef RigidBodyFrame<double> RigidBodyFramed;
typedef KinematicsCache<double> KinematicsCached;
typedef Matrix6X<double> Matrix6Xd;

const int kImageWidth = 640;  // In pixels
const int kImageHeight = 480;  // In pixels

int main() {
  // Create a formulation for a simple floating-base target
  string file_path = GetDrakePath() +
      "/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place.urdf";
  auto floating_base_type = multibody::joints::kRollPitchYaw;
  shared_ptr<RigidBodyFramed> weld_frame {nullptr};
  auto* mutable_tree_ = new RigidBodyTreed();
  parsers::urdf::AddModelInstanceFromUrdfFile(
          file_path, floating_base_type,
          weld_frame, mutable_tree_).begin();
  drake::multibody::AddFlatTerrainToWorld(mutable_tree_);
  mutable_tree_->compile();
  const auto* tree_ = mutable_tree_;

  // Add camera frame.
  const Vector3d position(-2, 0, 0.1);
  const Vector3d orientation(0, 0, 0); // degrees
  const double pi = M_PI;

  auto* world_body = const_cast<RigidBody<double>*>(&tree_->world());
  auto camera_frame_ = make_shared<RigidBodyFramed>(
      "depth_sensor", world_body, position, orientation * pi / 180);
  mutable_tree_->addFrame(camera_frame_);

  const double fov_y = pi / 4;
  auto* rgbd_camera_sim_ =
      new RgbdCameraDirect(*tree_, position, orientation, fov_y, true);
//      new RgbdCameraDirect(*tree_, *camera_frame_, fov_y, true);

  VectorXd x0(tree_->get_num_positions() + tree_->get_num_velocities());
  x0.setZero();
  x0(2) = 0.1;
  x0(5) = 10 * pi / 180;

  // Throw-away items.
  systems::rendering::PoseVector<double> pose;
  ImageBgra8U color_image(kImageWidth, kImageHeight);
  ImageLabel16I label_image(kImageWidth, kImageHeight);

  ImageDepth32F depth_image_meas(kImageWidth, kImageHeight);
  double t = 0;
  rgbd_camera_sim_->CalcImages(
      t, x0,
      &pose, &color_image, &depth_image_meas, &label_image);

  while (true) {
    timing::sleep(1);
  }
  return 0;
}

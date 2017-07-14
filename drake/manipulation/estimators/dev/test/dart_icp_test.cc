#include "drake/manipulation/estimators/dev/dart.h"
#include "drake/manipulation/estimators/dev/dart_objectives.h"
#include "drake/manipulation/estimators/dev/dart_depth_image_icp_objective.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/manipulation/estimators/dev/dart_util.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/multibody/rigid_body_tree_construction.h"

using namespace std;
using namespace drake::systems::sensors;

namespace drake {
namespace manipulation {
namespace {

// TODO(eric.cousineau): Consider mutable setup.
// TODO(eric.cousineau): Break into component-wise testing.
// TODO(eric.cousineau): Consider processing with VTK.
// TODO(eric.cousineau): Consider breaking MathematicalProgram into a
// Formulation and Solver components, such that it validates this and other
// designs. Make a DispatchSolver that can select from the variety at run-time,
// and can constraint on the solvers to choose.

struct Bounds2D {
  Interval x;
  Interval y;
};

struct IntervalIndex {
  int index;
  Interval interval;
};

struct PlaneIndices {
  IntervalIndex a;  // first plane coordinate
  IntervalIndex b;  // second plane coordinate
  IntervalIndex d;  // depth plane coordinate
};

Matrix2Xd GeneratePlane(double space, Interval x, Interval y) {
  const int max = floor(x.width() * y.width() / (space*space));
  int i = 0;
  Matrix2Xd out(2, max);
  for (int c = 0; c < x.width(); c++) {
    for (int r = 0; r < y.width(); r++) {
      out.col(i) << c * space, r * space;
      i++;
    }
  }
  out.conservativeResize(Eigen::NoChange, i);
  return out;
}

Matrix3Xd GeneratePlane(double space, PlaneIndices is) {
  // Generate single plane.
  Matrix2Xd p2d = GeneratePlane(space, is.a.interval, is.b.interval);
  // Map to 3d for upper and lower bound
  const int n = p2d.cols();
  Matrix3Xd p3du(3, 2 * n);
  auto map_into = [&](auto&& xpr, double value) {
    xpr.row(is.a.index) = p2d.row(0);
    xpr.row(is.b.index) = p2d.row(1);
    xpr.row(is.d.index).setConstant(value);
  };
  map_into(p3du.leftCols(n), is.d.interval.min);
  map_into(p3du.rightCols(n), is.d.interval.max);
  return p3du;
}

Matrix3Xd GenerateBoxPointCloud(double space, Bounds box) {
  IntervalIndex ix = {0, box.x};
  IntervalIndex iy = {1, box.y};
  IntervalIndex iz = {2, box.z};
  // Generate for each face
  auto xy_z = GeneratePlane(space, {ix, iy, iz});
  auto yz_x = GeneratePlane(space, {iy, iz, ix});
  auto xz_y = GeneratePlane(space, {ix, iz, iy});
  Matrix3Xd pts(3, xy_z.cols() + yz_x.cols() + xz_y.cols());
  pts << xy_z, yz_x, xz_y;
  return pts;
}

class DartIcpTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a formulation for a simple floating-base target
    InstanceIdMap instance_id_map;
    string file_path = FindResourceOrThrow(
        "drake/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place"
        ".urdf");
    auto floating_base_type = multibody::joints::kRollPitchYaw;
    shared_ptr<RigidBodyFramed> weld_frame {nullptr};
    mutable_tree_ = new RigidBodyTreed();
    instance_id_map["target"] =
        parsers::urdf::AddModelInstanceFromUrdfFile(
            file_path, floating_base_type,
            weld_frame, mutable_tree_).begin()->second;
    drake::multibody::AddFlatTerrainToWorld(mutable_tree_);
    mutable_tree_->compile();
    tree_.reset(mutable_tree_);

    scene_.reset(new DartScene(tree_, instance_id_map));

    DartFormulation::Param formulation_param {
      .estimated_positions = FlattenNameList({
          {"target", {"base_x", "base_y", "base_yaw"}} }),
    };
    formulation_.reset(
        new DartFormulation(CreateUnique(scene_), formulation_param));

    // Add camera frame.
    const Vector3d position(-2, 0, 0.1);
    const Vector3d orientation(0, 0, 0); // degrees
    const double pi = M_PI;

    auto* world_body = const_cast<RigidBody<double>*>(&tree_->world());
    camera_frame_.reset(new RigidBodyFramed(
        "depth_sensor", world_body, position, orientation * pi / 180));
    mutable_tree_->addFrame(camera_frame_);

    const double fov_y = pi / 4;
//    rgbd_camera_sim_.reset(
////        new RgbdCameraDirect(*tree_, *camera_frame_, fov_y, true));
//          new RgbdCameraDirect(*tree_, position, orientation * pi / 180, fov_y,
//                               true));

    const double fov_y = pi / 4;
    DartDepthImageIcpObjective::Param depth_param;
    {
      auto& param = depth_param;
      auto& camera = param.camera;
      camera = {
        .fov_y = fov_y,
      };
      camera.frame = camera_frame_;  // cannot be in initializer list.
      auto& icp = param.icp;
      icp = {
        .variance = 0.005,
      };
      auto& free_space = param.free_space;
      free_space.variance = 0.005;
      param.image_downsample_factor = 10;
      param.point_cloud_bounds = {
          .x = {-2, 2},
          .y = {-2, 2},
          .z = {-2, 2},
      };
    };
    depth_obj_ =
        new DartDepthImageIcpObjective(formulation_.get(), depth_param);
    formulation_->AddObjective(CreateUnique(depth_obj_));

    const Bounds box = {
      .x = {-0.03, 0.03},
      .y = {-0.03, 0.03},
      .z = {-0.1, 0.1},
    };
    const double space = 0.005;
    points_ = GenerateBoxPointCloud(space, box);
  }

  void TearDown() override {}

  void AddDepthObjective() {
  }

 protected:
  RigidBodyTreed* mutable_tree_;
  shared_ptr<const RigidBodyTreed> tree_;
  shared_ptr<RigidBodyFramed> camera_frame_;
  DartScene* scene_;
  unique_ptr<DartFormulation> formulation_;
  DartDepthImageIcpObjective* depth_obj_;
  Matrix3Xd points_;
};

TEST_F(DartIcpTest, BasicPointCloud) {
  // Feed box points initialized at (0, 0, 0), ensure that cost is (near)
  // zero.
  // Then, perturb measurement away from this, and ensure that error increases.

}

}  // namespace
}  // namespace manipulation
}  // namespace drake

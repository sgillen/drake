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

struct PointCloud {
  Matrix3Xd points;
  int size() const { return points.size(): }
};

struct PointCorrespondence {
  /** @brief Measured point index. */
  int meas_index{};
  // TODO(eric.cousineau): Consider storing index for descriptor information.
  /** @brief Model point, same frame as measured point. */
  Vector3d model_point;
  /** @brief Distance between the points. */
  double distance;
};

typedef map<BodyIndex, vector<PointCorrespondence>> SceneCorrespondence;

/**
 * This function depends on `q`.
 * @param scene
 * @param meas_pts_W
 * @param pcorrespondence
 */
void ComputeCorrespondences(const IcpScene& scene,
                            const Matrix3Xd& meas_pts_W,
                            SceneCorrespondence* pcorrespondence) {
  DRAKE_DEMAND(pcorrespondence != nullptr);
  int num_points = meas_pts_W.size();
  VectorXd distances(num_points);
  Matrix3Xd body_normals_W(3, num_points);  // world frame.
  Matrix3Xd body_pts_W(3, num_points);  // body point, world frame.
  Matrix3Xd body_pts_Bi(3, num_points);  // body point, body frame.
  vector<BodyIndex> body_indices(num_points);

  const bool use_margins = false;
  // TOOD(eric.cousineau): Figure out better access.
  RigidBodyTreed& mutable_tree =
      const_cast<RigidBodyTreed&>(scene.tree);
  mutable_tree.collisionDetectFromPoints(
      scene.cache, meas_pts_W,
      distances, body_normals_W,
      body_pts_W, body_pts_Bi,
      body_indices, use_margins);

  // Bin each correspondence to the given body.
  for (int i = 0; i < num_points; ++i) {
    BodyIndex body_index = body_indices[i];
    if (body_index != -1) {
      vector<PointCorrespondence>& body_correspondences =
          (*pcorrespondence)[body_index];
      PointCorrespondence pc{i, body_pts_W.col(i), distances[i]};
      body_correspondences.push_back(pc);
    }
  }
}

typedef std::function<void()> CostAccumulator;

double AccumulateCost(IcpScene& scene,
                      const Matrix3Xd& meas_pts_W,
                      const SceneCorrespondence& correspondence,
                      std::function<void()> accumulate) {
  for (const auto& pair : correspondence) {
    const BodyIndex body_index = pair.first;
    const vector<PointCorrespondence>& body_correspondence = pair.second;
    IcpPointGroup body_pts_group(body_index, body_correspondence.size());
    for (const auto& pc_W : body_correspondence) {
      body_pts_group.Add(meas_pts_W.col(pc_W.meas_index), pc_W.model_point);
    }
    CostAccumulator(body_pts_group);
  }
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

    scene_ = new DartScene(tree_, instance_id_map);

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

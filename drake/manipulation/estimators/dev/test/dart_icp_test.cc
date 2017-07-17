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
  int size() const { return points.size(); }
};

struct PointCorrespondence {
  // TODO(eric.cousineau): Consider storing normals, different frames, etc.
  /** @brief Measured point index. */
  int meas_index{};
  /** @brief Measured point. */
  Vector3d meas_point;
  /** @brief Model point, same frame as measured point. */
  Vector3d model_point;
  /** @brief Distance between the points. */
  double distance{-1};
};

typedef map<BodyIndex, vector<PointCorrespondence>> SceneCorrespondence;

struct BodyCorrespondenceInfluence {
  /**
   * @brief Will a correspondence with this body affect the camera
   * positions?
   */
  bool will_affect_camera{};
  /**
   * @brief Will a correspondence with this body affect the body's kinematics?
   */
  bool will_affect_body{};
  bool is_influential() const {
    return will_affect_camera || will_affect_body;
  }
};

void GetDofPath(const RigidBodyTreed& tree,
                BodyIndex body_index,
                std::vector<int> *pq_indices) {
  // NOTE: FindKinematicPath will only return joint information, but not DOF
  // information.
  auto& q_indices = *pq_indices;
  const int frame_W = 0;

  KinematicPath kinematic_path =
      tree.findKinematicPath(frame_W, body_index);
  int q_index = 0;
  // Adapted from: RigidBodyTree<>::geometricJacobian
  for (int cur_body_index : kinematic_path.joint_path) {
    const DrakeJoint& joint = tree.get_body(cur_body_index).getJoint();
    for (int i = 0; i < joint.get_num_positions(); ++i) {
      q_indices.push_back(q_index++);
    }
  }
}

template <typename ContainerA, typename ContainerB>
bool HasIntersection(const ContainerA& a, const ContainerB& b) {
  for (const auto& ai : a) {
    if (std::find(b.begin(), b.end(), ai) != b.end()) {
      return true;
    }
  }
  return false;
}

/**
 * Will a correspondence with a given body influence the optimization
 * formulation (camera or body position)? Will it mathematically affect
 * anything, or should computations relevant to this be skipped?
 * @param scene
 * @param ComputeCorrespondences
 */
BodyCorrespondenceInfluence IsBodyCorrespondenceInfluential(
    const IcpScene& scene,
    BodyIndex body_index,
    VectorSlice *position_slice) {
  BodyCorrespondenceInfluence out;
  out.will_affect_camera = (scene.frame_C != scene.frame_W);
  if (position_slice) {
    // Check if this body's kinematic chain will be affected by the slice.
    vector<int> q_indices;
    GetDofPath(scene.tree, body_index, &q_indices);
    // If there is any intersection, then this will influence things.
    out.will_affect_body =
        HasIntersection(q_indices, position_slice->indices());
  } else {
    out.will_affect_body = true;
  }
  return out;
}

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
      PointCorrespondence pc{
          i, meas_pts_W.col(i), body_pts_W.col(i), distances[i]};
      body_correspondences.push_back(pc);
    }
  }
}

typedef std::function<void(const IcpPointGroup&)> CostAggregator;

double AggregateCost(const IcpScene& scene,
                     const SceneCorrespondence& correspondence,
                     const CostAggregator& aggregate) {
  for (const auto& pair : correspondence) {
    const BodyIndex body_index = pair.first;
    const vector<PointCorrespondence>& body_correspondence = pair.second;
    IcpPointGroup body_pts_group(body_index, body_correspondence.size());
    for (const auto& pc_W : body_correspondence) {
      body_pts_group.Add(pc_W.meas_point, pc_W.model_point);
    }
    body_pts_group.Finalize();
    aggregate(body_pts_group);
  }
}

class QPCostAggregator {
 public:
  QPCostAggregator(const IcpScene& scene, double weight)
      : scene(&scene), weight(weight),
        error_accumulator(scene.tree.get_num_positions()) {}
  IcpLinearizedNormAccumulator error_accumulator;

  void operator()(const IcpPointGroup& body_pts_group) {
    body_pts_group.UpdateError(*scene, &es, &Jes);
    error_accumulator.AddTerms(weight, es, Jes);
  }
 private:
  const IcpScene* scene;
  double weight;
  Matrix3Xd es;
  MatrixXd Jes;
};

void AccumulateQPCost(const IcpScene& scene,
                      const SceneCorrespondence& correspondence,
                      double weight,
                      const VectorSlice& slice,
                      QuadraticCost* cost) {
  QPCostAggregator aggregator(scene, weight);
  AggregateCost(scene, correspondence, std::ref(aggregator));
  aggregator.error_accumulator.UpdateCost(slice, cost);
}

class DartIcpTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a formulation for a simple floating-base target
    string file_path = FindResourceOrThrow(
        "drake/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place"
        ".urdf");
    auto floating_base_type = multibody::joints::kRollPitchYaw;
    shared_ptr<RigidBodyFramed> weld_frame {nullptr};
    mutable_tree_ = new RigidBodyTreed();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        file_path, floating_base_type,
        weld_frame, mutable_tree_);
    drake::multibody::AddFlatTerrainToWorld(mutable_tree_);
    mutable_tree_->compile();
    tree_.reset(mutable_tree_);

//    DartFormulation::Param formulation_param {
//      .estimated_positions = FlattenNameList({
//          {"target", {"base_x", "base_y", "base_yaw"}} }),
//    };
    q_slice_.reset(new VectorSlice({0, 1, 5}, 6));

    // Add camera frame.
    const Vector3d position(-2, 0, 0.1);
    const Vector3d orientation(0, 0, 0); // degrees
    const double pi = M_PI;

    auto* world_body = const_cast<RigidBody<double>*>(&tree_->world());
    shared_ptr<RigidBodyFramed> camera_frame;
    camera_frame.reset(new RigidBodyFramed(
        "depth_sensor", world_body, position, orientation * pi / 180));
    mutable_tree_->addFrame(camera_frame);

    scene_.reset(new IcpScene(*tree_, *tree_cache_, 0,
                              camera_frame->get_frame_index()));

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
  unique_ptr<KinematicsCached> tree_cache_;
  shared_ptr<const RigidBodyTreed> tree_;
  unique_ptr<IcpScene> scene_;
  unique_ptr<VectorSlice> q_slice_;
  Matrix3Xd points_;
};

TEST_F(DartIcpTest, BasicPointCloud) {
  // Feed box points initialized at (0, 0, 0), ensure that cost is (near)
  // zero.
  // Then, perturb measurement away from this, and ensure that error increases.

  // Get correspondences
  SceneCorrespondence correspondence;
  ComputeCorrespondences(*scene_, points_, &correspondence);
  // Compute error
  int nvar = q_slice_->size();
  MatrixXd Q(nvar, nvar);
  VectorXd f(nvar);
  double c = 0;
  Q.setIdentity();
  f.setZero();
  solvers::QuadraticCost cost(Q, f, c);
  const double weight = 1;
  AccumulateQPCost(*scene_, correspondence, weight, *q_slice_, &cost);

  // The error should be near zero for points on the surface.
  EXPECT_EQ(0, cost.c());
}

}  // namespace
}  // namespace manipulation
}  // namespace drake

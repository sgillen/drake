#include "drake/perception/estimators/dev/articulated_icp.h"

#include <bot_core/pointcloud_t.hpp>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmtypes/drake/lcmt_viewer_draw.hpp"
#include "drake/lcmtypes/drake/lcmt_viewer_load_robot.hpp"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/create_load_robot_message.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/common/eigen_matrix_compare.h"

using std::make_shared;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;
using Eigen::Matrix2Xd;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Isometry3d;

namespace drake {
namespace perception {
namespace estimators {
namespace {

using solvers::MathematicalProgram;
using systems::BasicVector;
using systems::ViewerDrawTranslator;

const double kPi = M_PI;
//const double kQDiffNormMin = 0.01;

/**
 * Simple interval class.
 */
struct Interval {
  Interval() {}
  Interval(double min_in, double max_in)
      : min(min_in), max(max_in) {
    DRAKE_DEMAND(min <= max);
  }
  double min{};
  double max{};
  inline bool IsInside(double i) const { return i >= min && i <= max; }
  inline double width() const { return max - min; }
};

struct Bounds {
  Bounds() {}
  Bounds(Interval x_in, Interval y_in, Interval z_in)
      : x(x_in), y(y_in), z(z_in) {}
  Interval x;
  Interval y;
  Interval z;
  inline bool IsInside(double xi, double yi, double zi) const {
    return x.IsInside(xi) && y.IsInside(yi) && z.IsInside(zi);
  }
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

Matrix2Xd Generate2DPlane(double space, Interval x, Interval y) {
  const int nc = floor(x.width() / space);
  const int nr = floor(y.width() / space);
  int i = 0;
  Matrix2Xd out(2, nc * nr);
  for (int c = 0; c < nc; c++) {
    for (int r = 0; r < nr; r++) {
      out.col(i) << c * space + x.min, r * space + y.min;
      i++;
    }
  }
  out.conservativeResize(Eigen::NoChange, i);
  return out;
}

Matrix3Xd Generate2DPlane(double space, PlaneIndices is) {
  // Generate single plane.
  Matrix2Xd p2d = Generate2DPlane(space, is.a.interval, is.b.interval);
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
  auto xy_z = Generate2DPlane(space, {ix, iy, iz});
  auto yz_x = Generate2DPlane(space, {iy, iz, ix});
  auto xz_y = Generate2DPlane(space, {ix, iz, iy});
  Matrix3Xd pts(3, xy_z.cols() + yz_x.cols() + xz_y.cols());
  pts << xy_z, yz_x, xz_y;
  return pts;
}

/**
 * Computes estimated transform by aliging principle directions via principle
 * component analysis (PCA).
 * @param A A 3xn matrix represent `n` points in Cartesian space.
 * @param X_WA Transform of the assumed shape dictated by the points' first and
 * second moments (mean and covariance).
 */
Eigen::Isometry3d ComputePCATransform(const Matrix3Xd& A) {
  // http://www.cse.wustl.edu/~taoju/cse554/lectures/lect07_Alignment.pdf
  Vector3d A_mean = A.rowwise().mean();
  auto A_center = (A.colwise() - A_mean).eval();
  auto A_cov = (A_center * A_center.transpose()).eval();
  Eigen::Isometry3d X_WA;
  X_WA.setIdentity();
  X_WA.linear() = math::ProjectMatToRotMat(A_cov);
  X_WA.translation() = A_mean;
  return X_WA;
}

/**
 * Computes relative transformation to align points `a` with points `b`.
 * This requires that all correspondences be known.
 * @param p_B A 3xn matrix representing `n` points in the body frame,
 * {p_Bᵢ}ᵢ ∀ i ∈ {1..n}
 * @param y_W A 3xn matrix representing `n` measured points of the body, in
 * the world frame, {y_Wᵢ}ᵢ, where y_Wᵢ corresponds to p_Bᵢ.
 * @param X_AB Transformation from `B` to `W`, e.g., the pose of body `B` in
 * the world.
 */
Eigen::Isometry3d ComputeSVDTransform(
    const Matrix3Xd& p_B, const Matrix3Xd& y_W) {
  // TODO(eric.cousineau): Consider using Eigen::umeyama(), especially if
  // scaling is desired.
  // Following: http://cs.gmu.edu/~kosecka/cs685/cs685-icp.pdf
  // First moment: Take the mean of each point collection.
  Vector3d p_B_mean = p_B.rowwise().mean();
  Vector3d y_W_mean = y_W.rowwise().mean();
  // Compute deviations from mean.
  auto p_B_center = p_B.colwise() - p_B_mean;
  auto y_W_center = y_W.colwise() - y_W_mean;
  // Second moment: Compute covariance.
  Matrix3d W_py = p_B_center * y_W_center.transpose();
  // Compute SVD decomposition to reduce to a proper SO(3) basis.
  // TODO(eric.cousineau): If minimal cost from singular values is important,
  // consider accessing those values.
  Eigen::Isometry3d X_WB;
  X_WB.setIdentity();
  X_WB.linear() = math::ProjectMatToRotMat(W_py);
  X_WB.translation() = X_WB.linear() * p_B_mean - y_W_mean;
  return X_WB.inverse();
}

auto CompareTransforms(const Isometry3d& A, const Isometry3d& B,
                       double tolerance = 0.0) {
  return CompareMatrices(A.matrix(), B.matrix(), tolerance);
}

// TODO(eric.cousineau): Move to a proper LCM conversion type.
void PointCloudToLcm(const Matrix3Xd& pts_W, bot_core::pointcloud_t* pmessage) {
  bot_core::pointcloud_t& message = *pmessage;
  message.points.clear();
  message.frame_id = std::string(RigidBodyTreeConstants::kWorldName);
  message.n_channels = 0;
  message.n_points = pts_W.cols();
  message.points.resize(message.n_points);
  for (int i = 0; i < message.n_points; ++i) {
    Eigen::Vector3f pt_W = pts_W.col(i).cast<float>();
    message.points[i] = {pt_W(0), pt_W(1), pt_W(2)};
  }
  message.n_points = message.points.size();
}

class SimpleVisualizer {
 public:
  void PublishCloud(const Matrix3Xd& points, const string& suffix = "RGBD") {
    bot_core::pointcloud_t pt_msg;
    PointCloudToLcm(points, &pt_msg);
    vector<uint8_t> bytes(pt_msg.getEncodedSize());
    pt_msg.encode(bytes.data(), 0, bytes.size());
    lcm_.Publish("DRAKE_POINTCLOUD_" + suffix, bytes.data(), bytes.size());
  }

  void PublishFrames(const vector<Isometry3d>& frames) {
    drake::lcmt_viewer_draw msg{};
    const int num_frames = frames.size();
    msg.num_links = num_frames;
    msg.robot_num.resize(num_frames, 0);
    std::vector<float> pos = {0, 0, 0};
    std::vector<float> quaternion = {1, 0, 0, 0};
    msg.position.resize(num_frames, pos);
    msg.quaternion.resize(num_frames, quaternion);
    for (int i = 0; i < num_frames; ++i) {
      const auto& frame = frames[i];
      msg.link_name.push_back(fmt::format("frame_{}", i));
      for (int j = 0; j < 3; ++j) {
        msg.position[i][j] = static_cast<float>(frame.translation()[j]);
      }
      Quaternion<double> quat(frame.rotation());
      msg.quaternion[i][0] = static_cast<float>(quat.w());
      msg.quaternion[i][1] = static_cast<float>(quat.x());
      msg.quaternion[i][2] = static_cast<float>(quat.y());
      msg.quaternion[i][3] = static_cast<float>(quat.z());
    }
    vector<uint8_t> bytes(msg.getEncodedSize());
    msg.encode(bytes.data(), 0, bytes.size());
    lcm_.Publish("DRAKE_DRAW_FRAMES", bytes.data(), bytes.size());

    using namespace std;
    cout << "Published frame" << endl;
  }

  lcm::DrakeLcm& lcm() { return lcm_; }
 private:
  lcm::DrakeLcm lcm_;
};

GTEST_TEST(ArticulatedIcp, SVDAndPCA) {
  const Bounds box(
      Interval(-0.02, 0.02),
      Interval(-0.06, 0.06),
      Interval(-0.1, 0.1));
  using namespace std;
  cout << "Starting" << endl;
  SimpleVisualizer vis;

  const double spacing = 0.01;
  // Generate points in body frame.
  Matrix3Xd points_B = GenerateBoxPointCloud(spacing, box);
  // Expect identity on PCA.
  Isometry3d X_I;
  X_I.setIdentity();
  // Show the points in the world frame with identity transform.
  Matrix3Xd points_BI_W = X_I * points_B;
  vis.PublishCloud(points_BI_W, "BI");
  Isometry3d X_I_pca = ComputePCATransform(points_BI_W);
  const double tol = 1e-5;
  // Since our model is geometrically centered, this should be at or near
  // identity with PCA.
  EXPECT_TRUE(CompareTransforms(X_I, X_I_pca, spacing / 2));
  // Transform points.
  Isometry3d X_WB;
  X_WB.setIdentity();
  Vector3d xyz_B(0.5, 1, 1.5);
  Vector3d rpy_B(kPi / 10, kPi / 11, kPi / 12);
  X_WB.linear() << drake::math::rpy2rotmat(rpy_B);
  X_WB.translation() << xyz_B;
  // Transform.
  Matrix3Xd points_B_W = X_WB * points_B;
  vis.PublishCloud(points_B_W, "BW");
  // Compute PCA for each, and SVD for the pairs.
  Isometry3d X_WB_pca = ComputePCATransform(points_B_W);
  EXPECT_TRUE(CompareTransforms(X_WB, X_WB_pca, tol));
  // Compute SVD for both point sets.
  Isometry3d X_WB_svd = ComputeSVDTransform(points_B, points_B_W);
  EXPECT_TRUE(CompareTransforms(X_WB, X_WB_svd, tol));

  vis.PublishFrames({X_I, X_WB,
                     X_I_pca, X_WB_pca,
                     X_I * X_WB, X_I * X_WB_svd});
}

// TODO(eric.cousineau): Move to proper utility.
class IcpVisualizer : public SimpleVisualizer {
 public:
  explicit IcpVisualizer(const Scene* scene)
      : scene_(scene) {
    Init();
  }
  void Init() {
    const lcmt_viewer_load_robot load_msg(
        (multibody::CreateLoadRobotMessage<double>(scene_->tree())));
    vector<uint8_t> bytes(load_msg.getEncodedSize());
    load_msg.encode(bytes.data(), 0, bytes.size());
    lcm().Publish("DRAKE_VIEWER_LOAD_ROBOT", bytes.data(), bytes.size());
  }
  void PublishScene(const SceneState& scene_state) {
    const ViewerDrawTranslator draw_msg_tx(scene_->tree());
    drake::lcmt_viewer_draw draw_msg;
    vector<uint8_t> bytes;
    const int num_q = scene_->tree().get_num_positions();
    const int num_v = scene_->tree().get_num_velocities();
    const int num_x = num_q + num_v;
    BasicVector<double> x(num_x);
    auto xb = x.get_mutable_value();
    xb.setZero();
    xb.head(num_q) = scene_state.q();
    draw_msg_tx.Serialize(0, x, &bytes);
    lcm().Publish("DRAKE_VIEWER_DRAW", bytes.data(), bytes.size());
  }

 private:
  const Scene* scene_;
};

class ArticulatedIcpTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a formulation for a simple floating-base target
    string file_path = FindResourceOrThrow(
        "drake/perception/estimators/dev/simple_cuboid.urdf");
    // TODO(eric.cousineau): Use kQuaternion.
    auto floating_base_type = multibody::joints::kRollPitchYaw;
    shared_ptr<RigidBodyFramed> weld_frame{nullptr};
    mutable_tree_ = new RigidBodyTreed();
    parsers::urdf::AddModelInstanceFromUrdfFile(file_path, floating_base_type,
                                                weld_frame, mutable_tree_);

    mutable_tree_->compile();
    tree_.reset(mutable_tree_);
    tree_cache_.reset(new KinematicsCached(tree_->CreateKinematicsCache()));

    Vector3d obj_xyz(0, 0, 0.25);
    Vector3d obj_rpy(0, 0, 0);
    const int nq = tree_->get_num_positions();
    ASSERT_EQ(6, nq);
    q_init_.resize(nq);
    q_init_ << obj_xyz, obj_rpy;

    // Perturbation for initializing local ICP.
    q_perturb_.resize(nq);
    q_perturb_ << 0.3, 0.25, 0.25, kPi / 8, kPi / 12, kPi / 10;

    // Add fixed camera frame.
    // NOTE: At present, the camera pose does not influence anything given
    // that it is fixed-base.
    Vector3d camera_xyz(-2, 0, 0.1);
    camera_xyz += obj_xyz;
    Vector3d camera_rpy(0, 0, 0);  // degrees
    camera_rpy *= kPi / 180;
    auto* world_body = &mutable_tree_->world();
    shared_ptr<RigidBodyFramed> camera_frame;
    camera_frame.reset(
        new RigidBodyFramed("camera", world_body, camera_xyz, camera_rpy));
    mutable_tree_->addFrame(camera_frame);

    // Create the scene.
    scene_.reset(new Scene(tree_.get(), 0, camera_frame->get_frame_index()));

    // Compute the initial body correspondences.
    ComputeBodyCorrespondenceInfluences(*scene_, &influences_);

    // Generate simple point cloud corresponding to shape of box.
    const Bounds box(
        Interval(-0.03, 0.03),
        Interval(-0.03, 0.03),
        Interval(-0.1, 0.1));
    const double space = 0.02;
    // Transform to initial pose dictated by `q_init_`.
    Eigen::Isometry3d T_WB;
    T_WB.linear() << drake::math::rpy2rotmat(obj_rpy);
    T_WB.translation() << obj_xyz;
    points_ = T_WB * GenerateBoxPointCloud(space, box);

    // Create visualizer.
    vis_.reset(new IcpVisualizer(scene_.get()));
  }

 protected:
  RigidBodyTreed* mutable_tree_;
  unique_ptr<KinematicsCached> tree_cache_;
  shared_ptr<const RigidBodyTreed> tree_;
  VectorXd q_init_;
  VectorXd q_perturb_;
  unique_ptr<Scene> scene_;
  ArticulatedBodyInfluences influences_;
  Matrix3Xd points_;
  unique_ptr<IcpVisualizer> vis_;
};

#if false

TEST_F(ArticulatedIcpTest, PositiveReturnsZeroCost) {
  // Start box at the given state, ensure that the cost returned is near zero.
  VectorXd q = q_init_;
  SceneState scene_state(scene_.get());
  scene_state.Update(q);

  // Get correspondences
  ArticulatedPointCorrespondences correspondence;
  ComputeCorrespondences(scene_state, influences_, points_, &correspondence);
  // Compute error
  ArticulatedIcpErrorNormCost cost(scene_.get());
  ComputeCost(scene_state, correspondence, &cost);

  double tol = 1e-10;
  // The error should be near zero for points on the surface.
  EXPECT_NEAR(0, cost.cost(), tol);
  // Ensure that we have the same cost from the linearized L2 norm version.
  ArticulatedIcpLinearizedNormCost lin_cost(scene_.get());
  ComputeCost(scene_state, correspondence, &lin_cost);
  EXPECT_EQ(cost.cost(), lin_cost.cost());
}

// TODO(eric.cousineau): Add SVD computation for known correspondences.
// Add unittest comparing ICP for a single body using SVD.

TEST_F(ArticulatedIcpTest, PositiveReturnsIncreasingCost) {
  // Start box at the given state, ensure that the cost increases as we
  // translate away from the object given this simple scene.
  double prev_cost = 0;
  SceneState scene_state(scene_.get());
  for (int i = 0; i < 5; ++i) {
    VectorXd q = q_init_;
    q(0) = 0.05 * i;
    scene_state.Update(q);
    // Get correspondences
    ArticulatedPointCorrespondences correspondence;
    ComputeCorrespondences(scene_state, influences_, points_, &correspondence);
    // Compute error
    ArticulatedIcpErrorNormCost cost(scene_.get());
    ComputeCost(scene_state, correspondence, &cost);
    // Ensure cost is increasing.
    EXPECT_GT(cost.cost(), prev_cost);
    prev_cost = cost.cost();
    // Ensure that we have the same cost from the linearized L2 norm version.
    ArticulatedIcpLinearizedNormCost lin_cost(scene_.get());
    ComputeCost(scene_state, correspondence, &lin_cost);
    EXPECT_EQ(cost.cost(), lin_cost.cost());
  }
}

shared_ptr<solvers::QuadraticCost> MakeZeroQuadraticCost(int num_var) {
  return std::make_shared<solvers::QuadraticCost>(
      Eigen::MatrixXd::Zero(num_var, num_var),
      Eigen::VectorXd::Zero(num_var), 0);
}

shared_ptr<solvers::QuadraticCost> MakeConditioningCost(int num_var,
                                                        double value) {
  return std::make_shared<solvers::QuadraticCost>(
      value * Eigen::MatrixXd::Identity(num_var, num_var),
      Eigen::VectorXd::Zero(num_var), 0);
}

TEST_F(ArticulatedIcpTest, PositiveReturnsConvergenceTest) {
  // Test the number of iterations for the ICP to converge.
  const int num_q = tree_->get_num_positions();
  MathematicalProgram prog;
  const auto q_var = prog.NewContinuousVariables(num_q, "q");

  // Add cost for accumulating positive returns.
  auto qp_cost = MakeZeroQuadraticCost(num_q);
  prog.AddCost(qp_cost, q_var);
  // Add conditioning value for positive semi-definiteness.
  const double psd_cond = 1e-5;
  prog.AddCost(MakeConditioningCost(num_q, psd_cond), q_var);

  VectorXd q = q_init_ + q_perturb_;
  SceneState scene_state(scene_.get());

  const int iter_max = 15;

  int iter = 0;
  ArticulatedPointCorrespondences correspondence;
  ArticulatedIcpLinearizedNormCost icp_cost(scene_.get());

  while (true) {
    // Update formulation.
    scene_state.Update(q);

    correspondence.clear();
    ComputeCorrespondences(scene_state, influences_, points_, &correspondence);
    ComputeCost(scene_state, correspondence, &icp_cost);
    icp_cost.UpdateCost(qp_cost.get());

    // Solve.
    prog.SetInitialGuess(q_var, q);
    auto result = prog.Solve();
    ASSERT_EQ(solvers::kSolutionFound, result);

    const double q_diff_norm = (q - q_init_).norm();
    if (q_diff_norm < kQDiffNormMin) {
      break;
    }

    // Update initial guess.
    q = prog.GetSolution(q_var);
    ++iter;
    ASSERT_TRUE(iter < iter_max);
  }
  drake::log()->info("Took {} iterations for acceptable convergence", iter);

  scene_state.Update(q);
  vis_->PublishScene(scene_state);
  vis_->PublishCloud(points_);
}

#endif

}  // namespace
}  // namespace estimators
}  // namespace perception
}  // namespace drake

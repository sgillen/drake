#include "drake/perception/estimators/dev/articulated_icp.h"

#include <gtest/gtest.h>

#include <bot_core/pointcloud_t.hpp>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmtypes/drake/lcmt_viewer_draw.hpp"
#include "drake/lcmtypes/drake/lcmt_viewer_load_robot.hpp"
#include "drake/multibody/rigid_body_plant/create_load_robot_message.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/snopt_solver.h"

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

namespace drake {
namespace perception {
namespace estimators {
namespace {

const double kPi = M_PI;
const double kQDiffNormMin = 0.03;

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

void LoadVTPPointCloud(const std::string& filename, Matrix3Xd *pts) {
  auto reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(filename.c_str());
  reader->Update();

  // TODO(eric.cousineau): Consider using GetVoidPointer() and doing memory
  // mapping, akin to what the vtk.utils.numpy_support module does.
  vtkSmartPointer<vtkPolyData> polyData = reader->GetOutput();
  const int num_points = polyData->GetNumberOfPoints();
  pts->resize(Eigen::NoChange, num_points);
  Vector3d tmp;
  for (int i = 0; i < num_points; ++i) {
    polyData->GetPoint(i, tmp.data());
    pts->col(i) = tmp;
  }
}

class IcpVisualizer {
 public:
  IcpVisualizer(const Scene* scene, bool auto_init = true)
    : scene_(scene) {
    if (auto_init) {
      Init();
    }
  }
  void Init() {
    const lcmt_viewer_load_robot load_msg((
        multibody::CreateLoadRobotMessage<double>(scene_->tree())));
    vector<uint8_t> bytes(load_msg.getEncodedSize());
    load_msg.encode(bytes.data(), 0, bytes.size());
    lcm_.Publish("DRAKE_VIEWER_LOAD_ROBOT", bytes.data(), bytes.size());
  }
  void PublishScene(const SceneState& scene_state) {
    using namespace systems;
    const ViewerDrawTranslator draw_msg_tx(scene_->tree());
    drake::lcmt_viewer_draw draw_msg;
    vector<uint8_t> bytes;
    const int nq = scene_->tree().get_num_positions();
    const int nv = scene_->tree().get_num_velocities();
    const int nx = nq + nv;
    BasicVector<double> x(nx);
    auto xb = x.get_mutable_value();
    xb.setZero();
    xb.head(nq) = scene_state.q();
    draw_msg_tx.Serialize(0, x, &bytes);
    lcm_.Publish("DRAKE_VIEWER_DRAW", bytes.data(), bytes.size());
  }
  void PublishCloud(const Matrix3Xd& points) {
    bot_core::pointcloud_t pt_msg;
    PointCloudToLcm(points, &pt_msg);
    vector<uint8_t> bytes(pt_msg.getEncodedSize());
    pt_msg.encode(bytes.data(), 0, bytes.size());
    lcm_.Publish("DRAKE_POINTCLOUD_RGBD", bytes.data(), bytes.size());
  }
 private:
  lcm::DrakeLcm lcm_;
  const Scene* scene_;
};

//Eigen::Isometry3d xform(std::array<double, 3> pos,
//                        std::array<double, 4> quat) {
//  Eigen::Isometry3d out;
//  Eigen::Vector4d q(quat[0], quat[1], quat[2], quat[3]);
//  Eigen::Vector3d p(pos[0], pos[1], pos[2]);
//  out.translation() << p;
//  out.linear() << math::quat2rotmat(q);
//  return out;
//}

class ArticulatedIcpTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a formulation for a simple floating-base target
    string file_path = FindResourceOrThrow(
        "drake/perception/estimators/dev/test/blue_funnel.urdf");
    auto floating_base_type = multibody::joints::kRollPitchYaw;
    shared_ptr<RigidBodyFramed> weld_frame {nullptr};
    mutable_tree_ = new RigidBodyTreed();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        file_path, floating_base_type,
        weld_frame, mutable_tree_);

    mutable_tree_->compile();
    tree_.reset(mutable_tree_);
    tree_cache_.reset(new KinematicsCached(tree_->CreateKinematicsCache()));

    // Recorded point from captured CORL scene:
    //   f = om.findObjectByName("blue_funnel frame")
    //   print(f.transform)
    Eigen::Isometry3d X_WM;
    X_WM.linear() <<
      0.147663, 0.642632, -0.751811,
      0.988952, -0.10596, 0.103667,
      -0.013042, -0.758812, -0.651179;
    X_WM.translation() << -0.026111, 0.0496843, 0.548844;

    Vector3d obj_xyz(X_WM.translation());
    Vector3d obj_rpy = drake::math::rotmat2rpy(X_WM.rotation());
    const int nq = tree_->get_num_positions();
    ASSERT_EQ(6, nq);
    q_init_.resize(nq);
    q_init_ << obj_xyz, obj_rpy;

    // Perturbation for initializing local ICP.
    q_perturb_.resize(nq);
    q_perturb_.setConstant(0.02);
    //q_perturb_ << 0.3, 0.25, 0.25, kPi / 8, kPi / 12, kPi / 10;

    // Add fixed camera frame.
    Vector3d camera_xyz(-2, 0, 0.1);
    camera_xyz += obj_xyz;
    Vector3d camera_rpy(0, 0, 0); // degrees
    camera_rpy *= kPi / 180;

    auto* world_body = &mutable_tree_->world();
    shared_ptr<RigidBodyFramed> camera_frame;
    camera_frame.reset(new RigidBodyFramed(
        "camera", world_body, camera_xyz, camera_rpy));
    mutable_tree_->addFrame(camera_frame);

    // Create the scene.
    scene_.reset(
        new Scene(tree_.get(), 0, camera_frame->get_frame_index()));

    // Compute the initial body correspondences.
    ComputeBodyCorrespondenceInfluences(*scene_, &influences_);

    {
      string file_path = FindResourceOrThrow(
              "drake/perception/estimators/dev/test/blue_funnel_meas.vtp");
      LoadVTPPointCloud(file_path, &points_);
    }

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
//
//TEST_F(ArticulatedIcpTest, PositiveReturnsZeroCost) {
//  // Start box at the given state, ensure that the cost returned is near zero.
//  VectorXd q = q_init_;
//  SceneState scene_state(scene_.get());
//  scene_state.Update(q);
//
//  // Get correspondences
//  ArticulatedPointCorrespondences correspondence;
//  ComputeCorrespondences(scene_state, influences_, points_, &correspondence);
//  // Compute error
//  ArticulatedIcpErrorNormCost cost(scene_.get());
//  ComputeCost(scene_state, correspondence, &cost);
//
//  double tol = 1e-10;
//  // The error should be near zero for points on the surface.
//  EXPECT_NEAR(0, cost.cost(), tol);
//}
//
//TEST_F(ArticulatedIcpTest, PositiveReturnsIncreasingCost) {
//  // Start box at the given state, ensure that the cost increases as we
//  // translate away from the object given this simple scene.
//  double prev_cost = 0;
//  SceneState scene_state(scene_.get());
//  for (int i = 0; i < 5; ++i) {
//    VectorXd q = q_init_;
//    q(0) = 0.05 * i;
//    scene_state.Update(q);
//    // Get correspondences
//    ArticulatedPointCorrespondences correspondence;
//    ComputeCorrespondences(scene_state, influences_, points_, &correspondence);
//    // Compute error
//    ArticulatedIcpErrorNormCost cost(scene_.get());
//    ComputeCost(scene_state, correspondence, &cost);
//    // Ensure cost is increasing.
//    EXPECT_GT(cost.cost(), prev_cost);
//    prev_cost = cost.cost();
//  }
//}

shared_ptr<solvers::QuadraticCost> MakeZeroQuadraticCost(int nvar) {
  // Better way to do this?
  return std::make_shared<solvers::QuadraticCost>(
      Eigen::MatrixXd::Zero(nvar, nvar), Eigen::VectorXd::Zero(nvar), 0);
}

shared_ptr<solvers::QuadraticCost> MakeConditioningCost(
    int nvar, double value) {
  return std::make_shared<solvers::QuadraticCost>(
      value * Eigen::MatrixXd::Identity(nvar, nvar),
      Eigen::VectorXd::Zero(nvar), 0);
}

TEST_F(ArticulatedIcpTest, PositiveReturnsConvergenceTest) {
  // Test the number of iterations for the ICP to converge.
  using namespace drake::solvers;

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
    drake::log()->info("Iter {}\n\tq = [{}]", iter, q.transpose());
    // Update formulation.
    scene_state.Update(q);
    vis_->PublishScene(scene_state);
    vis_->PublishCloud(points_);

    correspondence.clear();
    ComputeCorrespondences(scene_state, influences_, points_, &correspondence);
    ComputeCost(scene_state, correspondence, &icp_cost);
    icp_cost.UpdateCost(qp_cost.get());

    // Solve.
    prog.SetInitialGuess(q_var, q);
    auto result = prog.Solve();
    ASSERT_EQ(kSolutionFound, result);

    const double q_diff_norm = (q - q_init_).norm();
    cout << q_diff_norm << endl;
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

}  // namespace
}  // namespace estimators
}  // namespace perception
}  // namespace drake

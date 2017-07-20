#include "drake/manipulation/estimators/dev/dart_icp.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmtypes/drake/lcmt_viewer_draw.hpp"
#include "drake/lcmtypes/drake/lcmt_viewer_load_robot.hpp"
#include "drake/multibody/rigid_body_plant/create_load_robot_message.h"
#include "bot_core/pointcloud_t.hpp"
#include "external/lcmtypes_bot2_core/lcmtypes/bot_core/pointcloud_t.hpp"
#include "drake/solvers/mathematical_program.h"
#include "drake/manipulation/estimators/dev/scoped_timer.h"

using namespace std;
using namespace drake::systems::sensors;

namespace drake {
namespace manipulation {
namespace {

// TODO(eric.cousineau): Consider processing with VTK.
// TODO(eric.cousineau): Consider breaking MathematicalProgram into a
// Formulation and Solver components, such that it validates this and other
// designs. Make a DispatchSolver that can select from the variety at run-time,
// and can constraint on the solvers to choose.

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

const double inf = std::numeric_limits<double>::infinity();
const double pi = M_PI;

class DartIcpTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a formulation for a simple floating-base target
    string file_path = FindResourceOrThrow(
        "drake/manipulation/estimators/dev/simple_cuboid.urdf");
    auto floating_base_type = multibody::joints::kRollPitchYaw;
    shared_ptr<RigidBodyFramed> weld_frame {nullptr};
    mutable_tree_ = new RigidBodyTreed();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        file_path, floating_base_type,
        weld_frame, mutable_tree_);
//    drake::multibody::AddFlatTerrainToWorld(mutable_tree_);

    mutable_tree_->compile();
    tree_.reset(mutable_tree_);
    tree_cache_.reset(new KinematicsCached(tree_->CreateKinematicsCache()));

    Vector3d obj_pos(0, 0, 0.25);
    q0_.resize(6);
    q0_ << obj_pos, Vector3d::Zero();
//    q0_.setZero();
//    q0_.head(3) << obj_pos;

//    DartFormulation::Param formulation_param {
//      .estimated_positions = FlattenNameList({
//          {"target", {"base_x", "base_y", "base_yaw"}} }),
//    };
    q_slice_.reset(new VectorSlice({0, 1, 5}, 6));

    // Add camera frame.
    Vector3d position(-2, 0, 0.1);
    position += obj_pos;
    const Vector3d orientation(0, 0, 0); // degrees

    auto* world_body = const_cast<RigidBody<double>*>(&tree_->world());
    shared_ptr<RigidBodyFramed> camera_frame;
    camera_frame.reset(new RigidBodyFramed(
        "depth_sensor", world_body, position, orientation * pi / 180));
    mutable_tree_->addFrame(camera_frame);

    scene_.reset(new IcpScene(*tree_, *tree_cache_, 0,
                              camera_frame->get_frame_index()));

    ComputeBodyCorrespondenceInfluences(*scene_, q_slice_.get(),
                                        &influences_);

    const Bounds box = {
      .x = {-0.03, 0.03},
      .y = {-0.03, 0.03},
      .z = {-0.1, 0.1},
    };
    const double space = 0.02;
    points_ = GenerateBoxPointCloud(space, box);
    points_.colwise() += obj_pos;
  }

 protected:
  RigidBodyTreed* mutable_tree_;
  unique_ptr<KinematicsCached> tree_cache_;
  shared_ptr<const RigidBodyTreed> tree_;
  VectorXd q0_;
  unique_ptr<IcpScene> scene_;
  unique_ptr<VectorSlice> q_slice_;
  Influences influences_;
  Matrix3Xd points_;
};

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

void Visualize(const IcpScene& scene, const Matrix3Xd& points) {
  lcm::DrakeLcm lcmc;
  using namespace systems;
  {
    const lcmt_viewer_load_robot load_msg(
        (multibody::CreateLoadRobotMessage<double>(scene.tree)));
    vector<uint8_t> bytes(load_msg.getEncodedSize());
    load_msg.encode(bytes.data(), 0, bytes.size());
    lcmc.Publish("DRAKE_VIEWER_LOAD_ROBOT", bytes.data(), bytes.size());
  }
  {
    const ViewerDrawTranslator draw_msg_tx(scene.tree);
    drake::lcmt_viewer_draw draw_msg;
    vector<uint8_t> bytes;
    const int nq = scene.tree.get_num_positions();
    const int nv = scene.tree.get_num_velocities();
    const int nx = nq + nv;
    BasicVector<double> x(nx);
    auto xb = x.get_mutable_value();
    xb.setZero();
    xb.head(nq) = scene.cache.getQ();
    draw_msg_tx.Serialize(0, x, &bytes);
    lcmc.Publish("DRAKE_VIEWER_DRAW", bytes.data(), bytes.size());
  }
  // Publish point cloud
  {
    bot_core::pointcloud_t pt_msg;
    PointCloudToLcm(points, &pt_msg);
    vector<uint8_t> bytes(pt_msg.getEncodedSize());
    pt_msg.encode(bytes.data(), 0, bytes.size());
    lcmc.Publish("DRAKE_POINTCLOUD_RGBD", bytes.data(), bytes.size());
  }
}

//TEST_F(DartIcpTest, PositiveReturnsZeroCost) {
//  // Start box at the given state, ensure that the cost returned is near zero.
//  VectorXd q0 = q0_;
//  tree_cache_->initialize(q0);
//  tree_->doKinematics(*tree_cache_);
//  if (true) {
//    Visualize(*scene_, points_);
//  }
//
//  // Get correspondences
//  IcpSceneCorrespondences correspondence;
//  ComputeCorrespondences(*scene_, points_, influences_, &correspondence);
//
//  // Compute error
//  IcpCostAggregator aggregator(*scene_);
//  AggregateCost(*scene_, correspondence, std::ref(aggregator));
//
//  double tol = 1e-10;
//  // The error should be near zero for points on the surface.
//  EXPECT_NEAR(0, aggregator.cost(), tol);
//}
//
//TEST_F(DartIcpTest, PositiveReturnsIncreasingCost) {
//  // Start box at the given state, ensure that the cost increases as we
//  // translate away from the object.
//  double prev_cost = 0;
//  for (int i = 0; i < 5; ++i) {
//    VectorXd q0 = q0_;
//    // NOTE: Due to local minima, cost will not decrease if moving along x,
//    // y, and z together.
//    q0(0) = 0.01 * i;
//    tree_cache_->initialize(q0);
//    tree_->doKinematics(*tree_cache_);
//    // Get correspondences
//    IcpSceneCorrespondences correspondence;
//    ComputeCorrespondences(*scene_, points_, influences_, &correspondence);
//    // Compute error
//    IcpCostAggregator aggregator(*scene_);
//    AggregateCost(*scene_, correspondence, std::ref(aggregator));
//    // The error should be near zero for points on the surface.
//    EXPECT_GT(aggregator.cost(), prev_cost);
//    prev_cost = aggregator.cost();
//  }
//}

shared_ptr<solvers::QuadraticCost> MakeZeroQuadraticCost(int nvar) {
  // Better way to do this?
  return make_shared<solvers::QuadraticCost>(
      Eigen::MatrixXd::Zero(nvar, nvar), Eigen::VectorXd::Zero(nvar), 0);
}
shared_ptr<solvers::QuadraticCost> MakeConditioningCost(
    int nvar, double value) {
  return make_shared<solvers::QuadraticCost>(
      value * Eigen::MatrixXd::Identity(nvar, nvar),
      Eigen::VectorXd::Zero(nvar), 0);
}

TEST_F(DartIcpTest, ConvergenceTest) {
  // Test the number of iterations for the ICP to converge.
  using namespace drake::solvers;
  MathematicalProgram prog;
  const int nq = tree_->get_num_positions();
  const auto q_var = prog.NewContinuousVariables(nq, "q");
  const VectorSlice q_slice(CardinalIndices(nq), nq);

  const int iter_max = 10;
  const double cost_min = 10;
  auto qp_cost = MakeZeroQuadraticCost(nq);
  prog.AddCost(qp_cost, q_var);
  // Add conditioning value.
  const double psd_cond = 1e-5;
  prog.AddCost(MakeConditioningCost(nq, psd_cond), q_var);

  VectorXd q_perturb(6);
  q_perturb << 4, 5, 6, pi / 8, pi / 6, pi / 5;
  VectorXd q0 = q0_ + q_perturb;
  cout << q0.transpose() << endl;

  int iter = 0;
  double cost = inf;
  while (cost > cost_min) {
    // Update formulation.
    tree_cache_->initialize(q0);
    tree_->doKinematics(*tree_cache_);

    Visualize(*scene_, points_);
    drake::manipulation::sleep(1);

    IcpSceneCorrespondences correspondence;
    ComputeCorrespondences(*scene_, points_, influences_, &correspondence);
    IcpLinearizedCostAggregator aggregator(*scene_);

    AggregateCost(*scene_, correspondence, std::ref(aggregator));
//    aggregator.PrintDebug(true);
    double cost_pre = aggregator.cost();

    aggregator.UpdateCost(q_slice, qp_cost.get());

//    cout << fmt::format("Q = [\n{}\n]\n\nf = [\n{}\n]\n\nc = {}\n\n",
//                        qp_cost->Q(), qp_cost->b(), qp_cost->c());

    // Solve.
    prog.SetInitialGuess(q_var, q0);
    auto result = prog.Solve();
    ASSERT_EQ(kSolutionFound, result);

    // Update initial guess.
    q0 = prog.GetSolution(q_var);

    double cost = prog.GetOptimalCost();
    VectorXd cost_2(1);
    qp_cost->Eval(q0, cost_2);

    cout << fmt::format("{}: {} -> {}, {}, {}\n", iter, cost_pre, cost,
                        cost_2(0), qp_cost->c());
    if (cost < cost_min) {
      break;
    }

    ++iter;
    ASSERT_TRUE(iter < iter_max);
  }

  tree_cache_->initialize(q0);
  tree_->doKinematics(*tree_cache_);
  Visualize(*scene_, points_);
  cout << q0.transpose() << endl;
  drake::manipulation::sleep(1);
}

}  // namespace
}  // namespace manipulation
}  // namespace drake

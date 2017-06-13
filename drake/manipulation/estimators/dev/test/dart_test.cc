#include "drake/manipulation/estimators/dev/dart.h"
#include "drake/manipulation/estimators/dev/dart_objectives.h"
#include "drake/manipulation/estimators/dev/dart_depth_image_icp_objective.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/manipulation/estimators/dev/dart_util.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/sensors/rgbd_camera.h"

using namespace std;
using namespace drake::systems::sensors;

namespace drake {
namespace manipulation {
namespace {

// TODO(eric.cousineau): Consider mutable setup.
// TODO(eric.cousineau): Break into component-wise testing.
// TODO(eric.cousineau): Consider breaking MathematicalProgram into a
// Formulation and Solver components, such that it validates this and other
// designs. Make a DispatchSolver that can select from the variety at run-time,
// and can constraint on the solvers to choose.

typedef vector<pair<string, vector<string>>> InstanceJointList;
vector<string> FlattenNameList(const InstanceJointList& joints) {
  vector<string> joints_flat;
  for (const auto& pair : joints) {
    const auto& instance_name = pair.first;
    const auto& instance_joints = pair.second;
    for (const auto& joint_name : instance_joints) {
      joints_flat.push_back(instance_name + "::" + joint_name);
    }
  }
  return std::move(joints_flat);
}

class DartTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a formulation for a simple floating-base target
    InstanceIdMap instance_id_map;
    string file_path = GetDrakePath() +
        "/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place.urdf";
    auto floating_base_type = multibody::joints::kRollPitchYaw;
    shared_ptr<RigidBodyFramed> weld_frame {nullptr};
    mutable_tree_ = new RigidBodyTreed();
    instance_id_map["target"] =
        parsers::urdf::AddModelInstanceFromUrdfFile(
            file_path, floating_base_type,
            weld_frame, mutable_tree_).begin()->second;
    mutable_tree_->compile();
    tree_.reset(mutable_tree_);

    scene_ =
        new DartScene(tree_, instance_id_map);

    // Only interested in x-y-yaw (planar position) of the block
    DartFormulation::Param formulation_param {
      .estimated_positions = FlattenNameList({
          {"target", {"base_x", "base_y", "base_yaw"}} }),
    };
    formulation_ =
        new DartFormulation(CreateUnique(scene_), formulation_param);
  }

  void TearDown() override {}

  void AddJointObjective() {
    DartJointObjective::Param joint_param = {
      .joint_variance = VectorXInit({0.05, 0.05, 0.01}),
    };
    joint_obj_ =
        new DartJointObjective(formulation_, joint_param);
    formulation_->AddObjective(CreateUnique(joint_obj_));
  }

  void AddDepthObjective() {
    // Add camera frame.
    const Vector3d position(-2, 0, 0);
    const Vector3d orientation(0, 0, 0); // degrees
    const double pi = M_PI;

    auto* world_body = const_cast<RigidBody<double>*>(&tree_->world());
    camera_frame_.reset(new RigidBodyFramed(
        "depth_sensor", world_body, position, orientation * pi / 180));
    mutable_tree_->addFrame(camera_frame_);

    const double fov_y = pi / 4;
    rgbd_camera_sim_.reset(
        new RgbdCameraDirect(*tree_, *camera_frame_, fov_y, true));

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
        new DartDepthImageIcpObjective(formulation_, depth_param);
    formulation_->AddObjective(CreateUnique(depth_obj_));
  }

  void CreateEstimator() {
    KinematicsState initial_state(scene_->tree());
    EXPECT_EQ(6, initial_state.q().size());
    EXPECT_EQ(6, initial_state.v().size());

    // Tie things together.
    DartEstimator::Param estimator_param {
      .initial_state = initial_state,
    };
    estimator_.reset(
        new DartEstimator(CreateUnique(formulation_), estimator_param));
  }

  void SimulateDepthImage(double t, const KinematicsState& state_meas,
                          ImageDepth32F* pdepth_image_meas) {
    // Throw-away items.
    systems::rendering::PoseVector<double> pose;
    ImageBgra8U color_image(kImageWidth, kImageHeight);
    ImageLabel16I label_image(kImageWidth, kImageHeight);

    rgbd_camera_sim_->CalcImages(
        t, state_meas.x(),
        &pose, &color_image, pdepth_image_meas, &label_image);
  }

  void SimulateObservation(double t, const KinematicsState& state_meas) {
    // Observe joint states.
    estimator_->ObserveAndInputKinematicsState(t, state_meas);
    if (joint_obj_) {
      joint_obj_->ObserveState(t, state_meas);
    }

    // Simulate depth image.
    if (depth_obj_) {
      ImageDepth32F depth_image_meas(kImageWidth, kImageHeight);
      SimulateDepthImage(t, state_meas, &depth_image_meas);
      depth_obj_->ObserveImage(t, depth_image_meas);
    }
  }

  KinematicsState Update(double t) {
    // Update estimate.
    return estimator_->Update(t);
  }

  KinematicsState UpdateAndCheckConvergence(double t,
                                 KinematicsState& state_prev,
                                 KinematicsState& state_meas) {
    SimulateObservation(t, state_meas);
    KinematicsState state_update = Update(t);

    // Check that we have gotten somewhat closer to our measured state, only in
    // the estimated coordinates.
    const KinematicsSlice& state_est_slice =
        formulation_->kinematics_est_slice();
    // Estimated portion of initial state.
    auto state_est_prev =
        state_est_slice.CreateFromSuperset(state_prev);
    // Estimated portion of measurement.
    auto state_est_meas =
        state_est_slice.CreateFromSuperset(state_meas);
    // Estimated portion of upate.
    auto state_est_update =
        state_est_slice.CreateFromSuperset(state_update);

    cout
        << "t: " << t << endl
        << "x_prev: " << state_est_prev.x().transpose() << endl
        << "x_meas: " << state_est_meas.x().transpose() << endl
        << "x_update: " << state_est_update.x().transpose() << endl;

    // Compute differences.
    auto diff_est_prev =
        (state_est_prev.x() - state_est_meas.x()).eval();
    auto diff_est_update =
        (state_est_update.x() - state_est_meas.x()).eval();
    const double eps = 1e-5;
    if (diff_est_update.norm() < eps && diff_est_prev.norm() < eps) {
      cout << "Converged." << endl;
    } else {
      EXPECT_LT(diff_est_update.norm(), diff_est_prev.norm())
          << "diff_est_update: " << diff_est_update.transpose() << endl
          << "diff_est_prev: " << diff_est_prev.transpose() << endl;
    }
    return state_update;
  }

  void CheckShortLoop() {
    KinematicsState state_prev = estimator_->initial_state();
    KinematicsState state_meas(*tree_);
    state_meas.q() <<
        1, 2, 0, 0, 0, 10 * M_PI / 180.;
    double t = 0;
    double dt = 0.01;
    double t_end = 0.05;
    while (t < t_end) {
      state_prev = UpdateAndCheckConvergence(t, state_prev, state_meas);
      t += dt;
    }
  }

 protected:
  RigidBodyTreed* mutable_tree_;
  shared_ptr<const RigidBodyTreed> tree_;
  shared_ptr<RigidBodyFramed> camera_frame_;
  DartScene* scene_;
  DartFormulation* formulation_;

  unique_ptr<RgbdCameraDirect> rgbd_camera_sim_;

  DartJointObjective* joint_obj_{};
  DartDepthImageIcpObjective* depth_obj_{};

  unique_ptr<DartEstimator> estimator_;
};

TEST_F(DartTest, JointObjective) {
  AddJointObjective();
  CreateEstimator();
  CheckShortLoop();
}

TEST_F(DartTest, DepthObjective) {
  AddDepthObjective();
  CreateEstimator();
  CheckShortLoop();
}

//TEST_F(DartTest, JointAndDepthObjective) {
//  AddJointObjective();
//  AddDepthObjective();
//  CreateEstimator();
//  CheckShortLoop();
//}

}  // namespace
}  // namespace manipulation
}  // namespace drake

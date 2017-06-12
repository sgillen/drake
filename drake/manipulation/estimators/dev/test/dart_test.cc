#include "drake/manipulation/estimators/dev/dart.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/manipulation/estimators/dev/dart_util.h"
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

class DartTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a formulation for a simple floating-base target
    string file_path = GetDrakePath() +
        "/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place.urdf";
    auto floating_base_type = multibody::joints::kRollPitchYaw;
    RigidBodyFramed* weld_frame = nullptr;
    tree_.reset(new RigidBodyTreed());
    parsers::urdf::AddModelInstanceFromUrdfFile(
        file_path, floating_base_type,
        weld_frame, tree_.get());
    tree_->compile();

    // Add camera frame.
    camera_frame_ = tree_->addFrame(...);

    rgbd_camera_sim_.reset(
        new RgbdCameraDirect(*tree_, camera_frame_*, ));

    VectorXd q0(tree_->get_num_positions());
    q0.setZero();
    VectorXd v0(tree_->get_num_velocities());
    v0.setZero();

    EXPECT_EQ(6, q0.size());
    EXPECT_EQ(6, v0.size());

    scene_ =
        new DartScene(tree_, q0, v0);

    // Only interested in x-y-yaw (planar position) of the block
    DartFormulation::Param formulation_param {
      .estimated_joints = {true, true, false, false, false, true},
    };
    formulation_ =
        new DartFormulation(CreateUnique(scene_), formulation_param);

    DartJointObjective::Param joint_param = {
      .joint_variance = {0.05, 0.05, 0.01},
    };
    joint_obj_ =
        new DartJointObjective(formulation_, joint_param);

    DartDepthImageIcpObjective::Param depth_param {
      .camera_frame = camera_frame_,
      .icp_variance = 0.05,
      .free_space_variance = 0.005,
      .downsample_factor = 5,
      .point_cloud_bounds = {
          .x = {-2, 2},
          .y = {-2, 2},
          .z = {-2, 2},
      },
    };
    depth_obj_ = DartDepthImageIcpObjective(formulation_, depth_param);

    // Tie things together.
    estimator_.reset(new DartEstimator(formulation_));
    estimator_->AddObjective(CreateUnique(joint_obj_));
    estimator_->AddObjective(CreateUnique(depth_obj_));
  }

  void TearDown() override {}

  void SimulateDepthImage(double t, const VectorXd& q, Image32F* pdepth_image) {
    const int nq = tree_->get_num_positions();
    const int nv = tree_->get_num_velocities();
    VectorXd x(nq + nv);
    x.setZero();
    x.head(nq) = q;

    // Throw-away items.
    systems::rendering::PoseVector<double> pose;
    ImageBgra8U color_image;
    ImageLabel16I label_image;

    rgbd_camera_sim_->CalcImages(
        t, x,
        &pose, &color_image, pdepth_image, &label_image);
  }

  void Observe(double t, const VectorXd& q_meas) {
    // Update estimator.
    estimator_->ObserveTime(t);

    // Observe joint states.
    VectorXd v(tree_->get_num_velocities());
    v.setZero();
    joint_obj_->ObserveState(t, q_meas, v);

    // Simulate depth image.
    ImageDepth32F depth_image_meas;
    SimulateDepthImage(t, q_meas, &depth_image);
    depth_obj_->ObserveImage(t, depth_image_meas);
  }

  void Update(double t, VectorXd* q) {
    // Update estimate.
    VectorXd v;
    estimator_->Update(t, q, &v);
  }
 private:
  shared_ptr<const RigidBodyTreed> tree_;
  RigidBodyFramed* camera_frame_;
  DartScene* scene_;
  DartFormulation* formulation_;

  unique_ptr<RgbdCameraDirect> rgbd_camera_sim_;

  DartJointObjective* joint_obj_;
  DartDepthImageIcpObjective* depth_obj_;

  unique_ptr<DartEstimator> estimator_;
};

TEST_F(DartTest, BasicSetup) {
  double t = 0;
  KinematicState state_meas(scene_->initial_state());
  KinematicState state_est(scene_->initial_state());

  estimator_->Observe(t, state_meas.q);
  estimator_->Update(t, &state_est.q);
}

}  // namespace
}  // namespace manipulation
}  // namespace drake

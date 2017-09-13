#include "drake/examples/kuka_iiwa_arm/dev/push_pick_place/push_and_pick_state_machine.h"

#include <chrono>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/math/rotation_matrix.h"
#include "drake/examples/kuka_iiwa_arm/dev/push_pick_place/push_and_pick_utils.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_utils.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmtypes/drake/lcmt_viewer_draw.hpp"

#include "drake/systems/sensors/image.h"

using Eigen::Vector3d;
using Eigen::Isometry3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;

namespace drake {

using systems::sensors::ImageRgb8U;
using systems::sensors::ImageDepth32F;

namespace examples {
namespace kuka_iiwa_arm {
namespace push_and_pick {
namespace {
using pick_and_place::WorldState;
using pick_and_place::IiwaMove;
using pick_and_place::WsgAction;

using manipulation::planner::ConstraintRelaxingIk;


using std::string;
using std::vector;
using std::pair;
using Eigen::Quaternion;
void PublishFrames(
    lcm::DrakeLcmInterface* lcm,
    const vector<pair<string, Isometry3d>>& frames,
    const std::string& channel_suffix = "") {
  drake::lcmt_viewer_draw msg{};
  const int num_frames = frames.size();
  msg.num_links = num_frames;
  msg.robot_num.resize(num_frames, 0);
  const vector<float> pos = {0, 0, 0};
  const vector<float> quaternion = {1, 0, 0, 0};
  msg.position.resize(num_frames, pos);
  msg.quaternion.resize(num_frames, quaternion);
  for (int i = 0; i < num_frames; ++i) {
    const string& name = frames[i].first;
    const auto& frame = frames[i].second;
    msg.link_name.push_back(name);
    for (int j = 0; j < 3; ++j) {
      msg.position[i][j] = static_cast<float>(frame.translation()[j]);
    }
    Quaternion<float> quat(frame.rotation().cast<float>());
    msg.quaternion[i][0] = quat.w();
    msg.quaternion[i][1] = quat.x();
    msg.quaternion[i][2] = quat.y();
    msg.quaternion[i][3] = quat.z();
  }
  vector<uint8_t> bytes(msg.getEncodedSize());
  msg.encode(bytes.data(), 0, bytes.size());
  lcm->Publish("DRAKE_DRAW_FRAMES" + channel_suffix,
               bytes.data(), bytes.size());
}



// Position the gripper 10cm above the object before grasp.
const double kPrePushHeightOffset = 0.18;
const double kPenetrationDepth = 0.08;
using pick_and_place::PlanSequenceMotion;
using pick_and_place::PlanStraightLineMotion;
using pick_and_place::ComputeGraspPose;
//const Vector3<double> kDesiredGraspPosition(0.228, -0.243, 0.7545);
const Vector3<double> kDesiredSidewaysGraspPosition(0.228, -0.27, 0.77045);
const Vector3<double> kDesiredGraspPosition(0.228, -0.243, 0.77045);
const Vector3<double> kDesiredGraspGripperPosition(0.228 + 0.23/2,
                                                   -0.375, //-0.356,
                                                   0.77045);
const Vector3<double> kLiftFromPickPosition(0.0, -0.625087, 1.7645);
const Matrix3<double> kDesiredGraspGripperOrientation(
    (Eigen::MatrixXd(3,3) <<
                       0, 0, 1,
                       0, 1, 0,
                      -1, 0, 0).finished());
const Matrix3<double> kDesiredGraspObjectOrientation(
    (Matrix3<double>::Identity()));
} // namespace

struct PushAndPickStateMachine::PerceptionData {
  PerceptionData(PerceptionBase* perception_in) {
    DRAKE_DEMAND(perception_in != nullptr);
    perception = perception_in;
    X_OOe.setIdentity();
    X_WD.setIdentity();
  }

  bool Update() {
    if (sensor_time > prev_sensor_time) {
      log()->info("Update image");
      perception->Update(
          sensor_time,
          depth_image,
          X_WD);
      prev_sensor_time = sensor_time;
      return true;
    } else {
      return false;
    }
  }

  void Process(const Isometry3d& X_WO) {
    log()->info("Processing perception");
    X_WOe = perception->EstimatePose();

    // Store pose from estimated (Oe, with error) to actual (O).
    X_OOe = X_WO.inverse() * X_WOe;
  }

  PerceptionBase* perception{};
  Isometry3<double> X_OOe;

  Isometry3<double> X_WOe;

  // Sensor readings.
  double sensor_time{-1};
  ImageDepth32F depth_image;
  Isometry3d X_WD;

  // Actual updates (for discretization).
  double prev_sensor_time{-1};
};

PushAndPickStateMachine::PushAndPickStateMachine(
    bool loop,
    PerceptionBase* perception)
    : next_place_location_(0),
      loop_(loop),
      state_(kOpenGripper), /*/kScanSweep), // kOpenGripper),*/
    // Position and rotation tolerances.  These were hand-tuned by
    // adjusting to tighter bounds until IK stopped reliably giving
    // results.
      tight_pos_tol_(0.005, 0.005, 0.005),
      tight_rot_tol_(0.05),
      loose_pos_tol_(0.05, 0.05, 0.05),
      loose_rot_tol_(0.5) {
  if (perception) {
    perception_data_.reset(new PerceptionData(perception));
  }
}

PushAndPickStateMachine::~PushAndPickStateMachine() {}

// -> ReadCamera
void PushAndPickStateMachine::ReadImage(
    const double time,
    const systems::sensors::ImageDepth32F& depth,
    const Eigen::Isometry3d& X_WD) {
  // Store it.
  if (perception_data_) {
    perception_data_->sensor_time = time;
    perception_data_->depth_image = depth;
    perception_data_->X_WD = X_WD;
  }
}

void PushAndPickStateMachine::Update(
    const WorldState& env_state_in,
    const IiwaPublishCallback& iiwa_callback,
    const WsgPublishCallback& wsg_callback,
    manipulation::planner::ConstraintRelaxingIk* planner,
    bool* camera_needed) {
  IKResults ik_res;
  std::vector<double> times;
  robotlocomotion::robot_plan_t stopped_plan{};
  stopped_plan.num_states = 0;

  const RigidBodyTree<double>& iiwa = planner->get_robot();

  // Hack.
  bool print_state = false;
  lcm::DrakeLcm lcm;

  // Permit modification from perception
  // (make it return X_WOe rather than X_WO).
  WorldState env_state = env_state_in;
  if (perception_data_) {
    env_state.mutable_object_pose().matrix() *=
        perception_data_->X_OOe.matrix();
    PublishFrames(&lcm,
        {
          {"actual", env_state_in.get_object_pose()},
          {"estimated", env_state.get_object_pose()},
        }, "_BOOK");
  }

  const double scan_dist = 0.65;  // m
  const double scan_theta_start = -M_PI / 6;  // rad
  // TODO(eric.cousineau): Figure out how to connect the trajectories, or use
  // the full ik traj stuff.
  const double scan_theta_end = 0; // scan_theta_start + 0.01; //M_PI / 6;  // rad
  const int scan_waypoints = 3;
  // Center position of table-top relative to world.
  // (Estimated from using measurement panel.)
  const Vector3<double> P_WTc(0.47442, -0.79625, 0.76600);
  // Emitting position of arc scan (Ao) relative to table-top center (Tc).
  const Vector3<double> P_TcAo(-0.1, 0.2, 0);
    //0, 0, 0);

  // Orientation adjustment from A to gripper (G).
  Matrix3<double> R_GA;
//  R_GA << Vector3d::UnitY(), Vector3d::UnitX(), -Vector3d::UnitZ();
  R_GA.setIdentity();

  // TODO(eric.cousineau): Orientation as a gaze constraint in IK.

  // Pose of arc scan (Ai) relative to world (W). Transformed to be in the
  // gripper frame (G, Gi).
  auto GetGripperScanPose = [=](double theta) {
    DRAKE_ASSERT(theta >= scan_theta_start && theta <= scan_theta_end);
    Vector3d P_WAo = P_WTc + P_TcAo;
    Vector3d P_AoAi_W(0, -scan_dist * sin(theta), scan_dist * cos(theta));
    Isometry3d X_WAi;
    X_WAi.setIdentity();
    X_WAi.translation() = P_WAo + P_AoAi_W;
    // Point down.
    Matrix3d R_WAi;
    R_WAi << -Vector3d::UnitZ(), Vector3d::UnitY(), Vector3d::UnitX();
    R_WAi *= Eigen::AngleAxis<double>(theta, Vector3d::UnitZ()).matrix();
    X_WAi.linear() = R_WAi;
    Isometry3d X_WGi;
    X_WGi.linear() = X_WAi.rotation() * R_GA.inverse();
    X_WGi.translation() = X_WAi.translation();
    return X_WGi;
  };

  // log()->info("Book pos: [\n  {}\n]",
  //     env_state_in.get_object_pose().translation().transpose());

  switch (state_) {
    // Opens the gripper.
    case kOpenGripper: {
      if (!wsg_act_.ActionStarted()) {
        lcmt_schunk_wsg_command msg;
        wsg_act_.OpenGripper(env_state, &msg);
        wsg_callback(&msg);

        drake::log()->info("kOpenGripper at {}",
                           env_state.get_iiwa_time());
      }

      if (wsg_act_.ActionFinished(env_state)) {

        Eigen::AngleAxisd object_pose(env_state.get_object_pose().linear());
        drake::log()->info("Axis angle of object : {}, {}",
                           object_pose.axis().transpose(),
                           object_pose.angle());

        //if(env_state.get_object_pose().linear())
        state_ = kScanApproach;
            //kApproachPreSidewaysYPush;
            //kApproachPreSidewaysPick;
            //kApproachPreSidewaysYPush;
            //kApproachPrePushRotate;
        wsg_act_.Reset();
      }
      break;
    }

    case kScanApproach: {
      if (!iiwa_move_.ActionStarted()) {
        log()->info("kScanApproach");
        // Schedule scan from a specified position.
        // Move by max velocity???

        // TODO(eric.cousineau): Not a fan of starting from initial config by
        // using IK given current pose...

        double t = 2;
        Isometry3d X_WG0 = env_state.get_iiwa_end_effector_pose();
        Isometry3d X_WGi0 = GetGripperScanPose(scan_theta_start);

        PublishFrames(&lcm,
            {
                {"X_WG0", X_WG0},
                {"X_WGi[0]", X_WGi0},
            }, "_TRAJ");

        bool res = PlanSequenceMotion(
            env_state.get_iiwa_q(), 2, t,
            {X_WG0, X_WGi0},
            loose_pos_tol_, loose_rot_tol_,
            planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa,
                              times, ik_res.q_sol,
                              &plan);
        iiwa_callback(&plan);
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kScanSweep;
        iiwa_move_.Reset();
      }
      break;
    }
    case kScanSweep: {
      if (!iiwa_move_.ActionStarted()) {
        print_state = true;

        log()->info("Camera on");
        *camera_needed = true;

        log()->info("kScanSweep");
        const double t = 1;
        std::vector<Isometry3d> scan_poses(scan_waypoints);
        std::vector<std::pair<string, Isometry3d>> frames;
        for (int i = 0; i < scan_waypoints; ++i) {
          const double theta =
              scan_theta_start + (scan_theta_end - scan_theta_start) *
                  i / (scan_waypoints - 1);
          scan_poses[i] = GetGripperScanPose(theta);
          frames.push_back({fmt::format("X_Gi[{}]", i),
                            scan_poses[i]});
        }

        PublishFrames(&lcm, frames, "_TRAJ");

        bool res = PlanSequenceMotion(
            env_state.get_iiwa_q(), 2, t,
            scan_poses,
            loose_pos_tol_, loose_rot_tol_,
              planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa,
                              times, ik_res.q_sol,
                              &plan);
        iiwa_callback(&plan);
      }

      // For each step, record each new image.
      if (perception_data_) {
        perception_data_->Update();
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        print_state = true;

        log()->info("Camera off");
        *camera_needed = false;

        state_ = kScanFinishAndProcess;
        iiwa_move_.Reset();
      }
      break;
    }
    case kScanFinishAndProcess: {
      log()->info("kScanFinishAndProcess");

      // Process the collected data.
      const Isometry3d X_WO = env_state_in.get_object_pose();
      if (perception_data_) {
        perception_data_->Process(X_WO);
      }

      state_ = kApproachPreSidewaysYPush;
      break;
    }

    case kApproachPrePushRotate: {
      // Approaches kPreGraspHeightOffset above the center of the object.
      if (!iiwa_move_.ActionStarted()) {

        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
        X_Wend_effector_1_ = ComputeTopDownPushPose(env_state.get_object_pose(), kPrePushHeightOffset);
        drake::log()->info("object pose : \n{}", env_state.get_object_pose().linear());
        drake::log()->info("desired gripper pose : \n{}", X_Wend_effector_1_.linear());
       // 2 seconds, no via points.
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 0, 2,
            X_Wend_effector_0_, X_Wend_effector_1_,
            loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachPrePushRotate at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kApproachPushRotate;
        iiwa_move_.Reset();
      }
      break;
    }
    case kApproachPushRotate :{
      // Approaches kPreGraspHeightOffset above the center of the object.
      if (!iiwa_move_.ActionStarted()) {
        drake::log()->info("Starting ApproachPushRotate");
        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
        X_Wend_effector_1_ = ComputeTopDownPushPose(env_state.get_object_pose(), kPenetrationDepth);
        // set rotation to ideal rotation right here.....
        // 2 seconds, no via points.
//        X_Wend_effector_1_.linear() = kDesiredGraspGripperOrientation;
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 0, 0.5,
            X_Wend_effector_0_, X_Wend_effector_1_,
            loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachPushRotate at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
          state_ = kPushRotate;
        iiwa_move_.Reset();
      }
      break;
    }

    case kPushRotate : {
      // Approaches kPreGraspHeightOffset above the center of the object.
      if (!iiwa_move_.ActionStarted()) {
        drake::log()->info("Starting ApproachPushRotate");
        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
        X_Wend_effector_1_ = ComputeTopDownPushPose(env_state.get_object_pose(), kPenetrationDepth);
        // set rotation to ideal rotation right here.....
        // 2 seconds, no via points.
        X_Wend_effector_1_.linear() = kDesiredGraspGripperOrientation;
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 2, 4,
            X_Wend_effector_0_, X_Wend_effector_1_,
            loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kPushRotate at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {

        // if target rotation not reached repeat.
        if(!IsOrientationClose(env_state.get_object_pose().linear(), kDesiredGraspObjectOrientation, 0.01)) {
          state_ = kApproachPrePushRotate;
        } else {
          state_ = kApproachPreSidewaysXPush;
        }
        iiwa_move_.Reset();
      }
      break;
    }
    case kApproachPreSidewaysXPush : {
// Approaches kPreGraspHeightOffset above the center of the object.
      if (!iiwa_move_.ActionStarted()) {
        drake::log()->info("Starting kApproachPreSidewaysXPush");
        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
        X_Wend_effector_1_ = ComputeEdgeXPushPose(
            env_state.get_object_pose(), kPrePushHeightOffset);
        // set rotation to ideal rotation right here.....
        // 2 seconds, no via points.
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 0, 1.0,
            X_Wend_effector_0_, X_Wend_effector_1_,
            loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachPreSidewaysXPush at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
          state_ = kApproachSidewaysXPush;
        iiwa_move_.Reset();
      }
      break;
    }
    case kApproachSidewaysXPush : {
// Approaches kPreGraspHeightOffset above the center of the object.
      if (!iiwa_move_.ActionStarted()) {
        drake::log()->info("Starting kApproachSidewaysXPush");
        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
        X_Wend_effector_1_ = ComputeEdgeXPushPose(
            env_state.get_object_pose(), kPenetrationDepth);

        drake::log()->info("kApproachSidewaysXPush Original \n {}\n Target \n{}\n Object \n {}",
                           X_Wend_effector_0_.translation().transpose(),
                           X_Wend_effector_1_.translation().transpose(),
                           env_state.get_object_pose().translation().transpose());


        // set rotation to ideal rotation right here.....
        // 2 seconds, no via points.
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 0, 0.5,
            X_Wend_effector_0_, X_Wend_effector_1_,
            loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachSidewaysXPush at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        if(IsPositionXYClose(env_state.get_object_pose().translation(),
                             kDesiredSidewaysGraspPosition, 0.01)) {
          state_ = kApproachPreSidewaysPick;
        } else {
          state_=kSidewaysXPushMove;
        iiwa_move_.Reset();
      }
      break;
    }
    case kSidewaysXPushMove : {
      // Approaches kPreGraspHeightOffset above the center of the object.
      if (!iiwa_move_.ActionStarted()) {
        drake::log()->info("Starting kSidewaysXPushMove");
        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
        X_Wend_effector_1_.translation() = kDesiredGraspGripperPosition;
        X_Wend_effector_1_.translation()[2] = X_Wend_effector_0_.translation()[2];
        X_Wend_effector_1_.translation()[0] = X_Wend_effector_0_.translation()[0] - 0.02;
        X_Wend_effector_1_.translation()[1] += 0.05;
        X_Wend_effector_1_.translation()[2] -= 0.05;
        //X_Wend_effector_1_.translation()[1] -= -0.1;
        // set rotation to ideal rotation right here.....
        // 2 seconds, no via points.

        drake::log()->info("SidewaysXPushMove Original \n {}\n Target \n{}\n Object \n {}",
                           X_Wend_effector_0_.translation().transpose(),
                           X_Wend_effector_1_.translation().transpose(),
                           env_state.get_object_pose().translation().transpose());



        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 3, 2,
            X_Wend_effector_0_, X_Wend_effector_1_,
            loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kSidewaysPushMove at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kRiseFromSidewaysXPushMove;
        iiwa_move_.Reset();
      }
      break;
    }
      case kRiseFromSidewaysXPushMove : {
        // Approaches kPreGraspHeightOffset above the center of the object.
        if (!iiwa_move_.ActionStarted()) {
          drake::log()->info("Starting kRiseFromSidewaysYPushMove");
          // Computes the desired end effector pose in the world frame to be
          // kPreGraspHeightOffset above the object.
          X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
          X_Wend_effector_1_ = X_Wend_effector_0_;
          X_Wend_effector_1_.translation()[2] += kPrePushHeightOffset;
          // X_Wend_effector_1_.linear() = kDesiredGraspGripperOrientation;


          drake::log()->info("kRiseFromSidewaysYPushMove Original \n {}\n Target \n{}\n Object \n {}",
                             X_Wend_effector_0_.translation().transpose(),
                             X_Wend_effector_1_.translation().transpose(),
                             env_state.get_object_pose().translation().transpose());


          // set rotation to ideal rotation right here.....
          // 2 seconds, no via points.
          bool res = PlanStraightLineMotion(
              env_state.get_iiwa_q(), 0, 0.5,
              X_Wend_effector_0_, X_Wend_effector_1_,
              loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
          DRAKE_DEMAND(res);

          robotlocomotion::robot_plan_t plan{};
          iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
          iiwa_callback(&plan);

          drake::log()->info("kRiseFromSidewaysYPushMove at {}",
                             env_state.get_iiwa_time());
        }

        if (iiwa_move_.ActionFinished(env_state)) {
        if(IsPositionXYClose(env_state.get_object_pose().translation(), kDesiredGraspPosition, 0.01)) {
          state_ = kApproachPreSidewaysPick;
        } else {
          state_=kApproachPreSidewaysYPush;
        }
          iiwa_move_.Reset();
        }
        break;
      }
    case kApproachPreSidewaysYPush : {
      if (!iiwa_move_.ActionStarted()) {
        drake::log()->info("Starting kApproachReSidewaysYPush");
        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
        X_Wend_effector_1_ = ComputeEdgeYPushPose(
            env_state.get_object_pose(), kPrePushHeightOffset );
        // set rotation to ideal rotation right here.....
        // 2 seconds, no via points.
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 0, 0.5,
            X_Wend_effector_0_, X_Wend_effector_1_,
            loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachPreSidewaysYPush at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kApproachSidewaysYPush;
        iiwa_move_.Reset();
      }
      break;
    }
    case kApproachSidewaysYPush : {
      if (!iiwa_move_.ActionStarted()) {
        drake::log()->info("Starting kApproachSidewaysYPush");
        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
        X_Wend_effector_1_ = ComputeEdgeYPushPose(
            env_state.get_object_pose(), kPenetrationDepth );

        drake::log()->info("kApproachSidewaysYPush Original \n {}\n Target \n{}\n Object \n {}",
                           X_Wend_effector_0_.translation().transpose(),
                           X_Wend_effector_1_.translation().transpose(),
                           env_state.get_object_pose().translation().transpose());

        // set rotation to ideal rotation right here.....
        // 2 seconds, no via points.
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 0, 0.5,
            X_Wend_effector_0_, X_Wend_effector_1_,
            loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachSidewaysYPush at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kSidewaysYPushMove;
        iiwa_move_.Reset();
      }
      break;
    }
    case kSidewaysYPushMove : {
      // Approaches kPreGraspHeightOffset above the center of the object.
      if (!iiwa_move_.ActionStarted()) {
        drake::log()->info("Starting SidewaysYPushMove");
        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
        X_Wend_effector_1_.translation() = kDesiredGraspGripperPosition;
        X_Wend_effector_1_.translation()[2] = X_Wend_effector_0_.translation()[2];
        X_Wend_effector_1_.translation()[1] =
           0.5*( X_Wend_effector_0_.translation()[1] +
               X_Wend_effector_1_.translation()[1]);
        X_Wend_effector_1_.translation()[0] -= 0.01;
        X_Wend_effector_1_.linear() = kDesiredGraspGripperOrientation;


        drake::log()->info("SidewaysYPushMove Original \n {}\n Target \n{}\n Object \n {}",
                           X_Wend_effector_0_.translation().transpose(),
                           X_Wend_effector_1_.translation().transpose(),
                           env_state.get_object_pose().translation().transpose());


        // set rotation to ideal rotation right here.....
        // 2 seconds, no via points.
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 2, 2,
            X_Wend_effector_0_, X_Wend_effector_1_,
            loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("SidewaysYPushMove at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kRiseFromSidewaysYPushMove;
        iiwa_move_.Reset();
      }
      break;
    }
      case kRiseFromSidewaysYPushMove : {
        // Approaches kPreGraspHeightOffset above the center of the object.
        if (!iiwa_move_.ActionStarted()) {
          drake::log()->info("Starting kRiseFromSidewaysYPushMove");
          // Computes the desired end effector pose in the world frame to be
          // kPreGraspHeightOffset above the object.
          X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
          X_Wend_effector_1_ = X_Wend_effector_0_;
          X_Wend_effector_1_.translation()[2] += kPrePushHeightOffset;
         // X_Wend_effector_1_.linear() = kDesiredGraspGripperOrientation;


          drake::log()->info("kRiseFromSidewaysYPushMove Original \n {}\n Target \n{}\n Object \n {}",
                             X_Wend_effector_0_.translation().transpose(),
                             X_Wend_effector_1_.translation().transpose(),
                             env_state.get_object_pose().translation().transpose());


          // set rotation to ideal rotation right here.....
          // 2 seconds, no via points.
          bool res = PlanStraightLineMotion(
              env_state.get_iiwa_q(), 0, 0.5,
              X_Wend_effector_0_, X_Wend_effector_1_,
              loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
          DRAKE_DEMAND(res);

          robotlocomotion::robot_plan_t plan{};
          iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
          iiwa_callback(&plan);

          drake::log()->info("kRiseFromSidewaysYPushMove at {}",
                             env_state.get_iiwa_time());
        }

        if (iiwa_move_.ActionFinished(env_state)) {
          if(IsPositionXYClose(env_state.get_object_pose().translation(), kDesiredGraspPosition, 0.01)) {
            state_ = kApproachPreSidewaysPick;
          } else {
            state_=kApproachPreSidewaysXPush;
          }
          iiwa_move_.Reset();
        }
        break;
      }
    case kSidewaysXPushRotate : {
      break;
    }
    case kSidewaysYPushRotate : {
      break;
    }
    case kApproachPreSidewaysPick : {

      if (!iiwa_move_.ActionStarted()) {
        drake::log()->info("Starting kApproachPreSidewaysPick");
        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
        X_Wend_effector_1_ =ComputeSidewaysGraspPose(env_state.get_object_pose(), 0.1, 0.0 *M_PI);
        X_Wend_effector_1_.translation()[2] += 0.2;
        // set rotation to ideal rotation right here.....
        // 2 seconds, no via points.
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 0, 1,
            X_Wend_effector_0_, X_Wend_effector_1_,
            loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachPreSidewaysPick at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
          state_=kApproachSidewaysPick;
        iiwa_move_.Reset();
      }

      break;
    }
    case kApproachSidewaysPick : {
      if (!iiwa_move_.ActionStarted()) {
        drake::log()->info("Starting kApproachSidewaysPick");
        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
//        X_Wend_effector_1_ = ComputeSidewaysGraspPose(env_state.get_object_pose(), 0.165, -0.25 * M_PI);
//        X_Wend_effector_1_.translation()[2] += 0.065;

        X_Wend_effector_1_ = ComputeSidewaysGraspPose(env_state.get_object_pose(), 0.145, -0.32 * M_PI);
        X_Wend_effector_1_.translation()[2] += 0.0625;


        drake::log()->info("kApproachSidewaysPick Original \n {}\n Target \n{}\n Object \n {}",
                           X_Wend_effector_0_.translation().transpose(),
                           X_Wend_effector_1_.translation().transpose(),
                           env_state.get_object_pose().translation().transpose());

        drake::log()->info("kApproachSidewaysPick Ini Pose for IK \n{}, \nDesired pose for IK \n{}",
                           X_Wend_effector_0_.linear(),
                           X_Wend_effector_1_.linear());

        // set rotation to ideal rotation right here.....
        // 2 seconds, no via points.
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 0, 0.5,
            X_Wend_effector_0_, X_Wend_effector_1_,
            loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachSidewaysPick at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_=kCloseGripper;
        iiwa_move_.Reset();
      }
    }
      break;
    }
    case kCloseGripper : {{
        if (!wsg_act_.ActionStarted()) {
          lcmt_schunk_wsg_command msg;
          wsg_act_.CloseGripper(env_state, &msg);
          wsg_callback(&msg);

          drake::log()->info("kCloseGripper at {}",
                             env_state.get_iiwa_time());
        }

        if (wsg_act_.ActionFinished(env_state)) {
          state_ = kLiftFromPick;
          //kApproachPrePushRotate;
          wsg_act_.Reset();
        }
        break;
      }
    }
    case kLiftFromPick : {
      if (!iiwa_move_.ActionStarted()) {
        drake::log()->info("Starting kLiftFromPick");
        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
        X_Wend_effector_1_ =  ComputeSidewaysGraspPose(env_state.get_object_pose(), -0.55, -0.55 * M_PI);
        X_Wend_effector_1_.translation()[2] += 0.375;
        X_Wend_effector_1_.translation()[0] = 0.2;
        X_Wend_effector_1_.translation()[1] = -0.43;
        //X_Wend_effector_1_.linear() =kDesiredSiredGraspGripperOrientation;
            //kDesiredGraspGripperOrientation;

        //X_Wend_effector_1_.translation() = kLiftFromPickPosition;
        //X_Wend_effector_1_.linear() = kDesiredGraspGripperOrientation;
        // set rotation to ideal rotation right here.....
        // 2 seconds, no via points.
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 0, 2,
            X_Wend_effector_0_, X_Wend_effector_1_,
            loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kLiftFromPick at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_=kDone;
        iiwa_move_.Reset();
      }
      break;
    }
    case kDone: {
      break;
    }
  }

  if (print_state) {
    log()->info("q_iiwa: [\n  {}\n]", env_state.get_iiwa_q().transpose());
    log()->info("q_wsg: [\n  {}\n]", env_state.get_wsg_q());
  }
}

}  // namespace push_and_pick
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

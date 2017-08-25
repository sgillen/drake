#include "drake/examples/kuka_iiwa_arm/dev/push_pick_place/push_and_pick_state_machine.h"

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/math/rotation_matrix.h"
#include "drake/examples/kuka_iiwa_arm/dev/push_pick_place/push_and_pick_utils.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_utils.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace push_and_pick {
namespace {
using pick_and_place::WorldState;
using pick_and_place::IiwaMove;
using pick_and_place::WsgAction;

using manipulation::planner::ConstraintRelaxingIk;

// Position the gripper 10cm above the object before grasp.
const double kPreGraspHeightOffset = 0.1;
const double kPenetrationDepth = 0.14;
using pick_and_place::PlanStraightLineMotion;
using pick_and_place::ComputeGraspPose;
const Vector3 kDesiredGraspPosition();
} // namespace

PushAndPickStateMachine::PushAndPickStateMachine(bool loop)
    : next_place_location_(0),
      loop_(loop),
      state_(kOpenGripper),
    // Position and rotation tolerances.  These were hand-tuned by
    // adjusting to tighter bounds until IK stopped reliably giving
    // results.
      tight_pos_tol_(0.005, 0.005, 0.005),
      tight_rot_tol_(0.05),
      loose_pos_tol_(0.05, 0.05, 0.05),
      loose_rot_tol_(0.5) {
}

PushAndPickStateMachine::~PushAndPickStateMachine() {}

void PushAndPickStateMachine::Update(
    const WorldState& env_state,
    const IiwaPublishCallback& iiwa_callback,
    const WsgPublishCallback& wsg_callback,
    manipulation::planner::ConstraintRelaxingIk* planner) {
  IKResults ik_res;
  std::vector<double> times;
  robotlocomotion::robot_plan_t stopped_plan{};
  stopped_plan.num_states = 0;

  const RigidBodyTree<double>& iiwa = planner->get_robot();

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
        state_ = kApproachPreVerticalPush;
        wsg_act_.Reset();
      }
      break;
    }

    case kApproachPreVerticalPush: {
      // Approaches kPreGraspHeightOffset above the center of the object.
      if (!iiwa_move_.ActionStarted()) {

        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
        X_Wend_effector_1_ = ComputeTopDownPushPose(env_state.get_object_pose());
       // drake::log()->info("translation target re offset = {}", X_Wend_effector_1_.translation().transpose());
        X_Wend_effector_1_.translation()[2] += kPreGraspHeightOffset;
        X_Wend_effector_1_.translation()[1] += 0.0;//0.5;
        X_Wend_effector_1_.translation()[0] += 0.0; //0.4;
        //drake::log()->info("translation target = {}", X_Wend_effector_1_.translation().transpose());

        // 2 seconds, no via points.
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 0, 2,
            X_Wend_effector_0_, X_Wend_effector_1_,
            loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachPickPregrasp at {}",
                           env_state.get_iiwa_time());

        Isometry3<double> temp_pose = env_state.get_iiwa_end_effector_pose();
        drake::log()->info("Final position {}",
                           temp_pose.translation().transpose());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kApproachVerticalPush;//kApproachPick;
        iiwa_move_.Reset();
      }
      break;
    }

    case kApproachVerticalPush :{
      // Approaches kPreGraspHeightOffset above the center of the object.
      if (!iiwa_move_.ActionStarted()) {
        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
        X_Wend_effector_1_ = ComputeTopDownPushPose(env_state.get_object_pose());

        // 2 seconds, no via points.
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 0, 2.5,
            X_Wend_effector_0_, X_Wend_effector_1_,
            loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachVerticalPush at {}",
                           env_state.get_iiwa_time());

        Isometry3<double> temp_pose = env_state.get_iiwa_end_effector_pose();
        drake::log()->info("Final position {}",
                           temp_pose.translation().transpose());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kVerticalPushMove;//kApproachPick;
        iiwa_move_.Reset();
      }
      break;
    }
    case kVerticalPushMove : {
      // Approaches kPreGraspHeightOffset above the center of the object.
      if (!iiwa_move_.ActionStarted()) {

        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();

        drake::log()->info("object orientation \n{}", env_state.get_object_pose().linear());

        drake::log()->info("previous orientation \n{}", X_Wend_effector_0_.linear());
        X_Wend_effector_1_ = Isometry3<double>::Identity();
        X_Wend_effector_1_.linear()= Eigen::Matrix3d::Identity(); // ComputeTopDownPushPose(env_state.get_object_pose());
            //Eigen::AngleAxisd( M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::Matrix3d::Identity(); // ComputeTopDownPushPose(env_state.get_object_pose());
        drake::log()->info("current desired orientation \n{}",
                           X_Wend_effector_1_.linear());
        // drake::log()->info("translation target re offset = {}", X_Wend_effector_1_.translation().transpose());
        X_Wend_effector_1_.translation()[2] = 1.5;
        X_Wend_effector_1_.translation()[1] = 0;//0.5;
        X_Wend_effector_1_.translation()[0] = 0; //0.4;
        //drake::log()->info("translation target = {}", X_Wend_effector_1_.translation().transpose());

        // 2 seconds, no via points.
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 3, 10,
            X_Wend_effector_0_, X_Wend_effector_1_,
            loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kVerticalPushMove at {}",
                           env_state.get_iiwa_time());

        Isometry3<double> temp_pose = env_state.get_iiwa_end_effector_pose();
        drake::log()->info("Final position {}",
                           temp_pose.translation().transpose());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        Isometry3<double> current_object_pose = env_state.get_object_pose();
        if(current_object_pose.translation()[1] < -0.1) {
          state_ = kApproachPreVerticalPush;
        } else {
          state_ = kDone;//kApproachPick;
        }
        iiwa_move_.Reset();
      }
      break;
    }

//
//    case kApproachPick: {
//      // Moves gripper straight down.
//      if (!iiwa_move_.ActionStarted()) {
//        X_Wend_effector_0_ = X_Wend_effector_1_;
//        X_Wend_effector_1_ = ComputeGraspPose(env_state.get_object_pose());
//
//        // 2 seconds, 3 via points. More via points to ensure the end
//        // effector moves in more or less a straight line.
//        bool res = PlanStraightLineMotion(
//            env_state.get_iiwa_q(), 3, 2,
//            X_Wend_effector_0_, X_Wend_effector_1_,
//            tight_pos_tol_, tight_rot_tol_, planner, &ik_res, &times);
//        DRAKE_DEMAND(res);
//
//        robotlocomotion::robot_plan_t plan{};
//        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
//        iiwa_callback(&plan);
//
//        drake::log()->info("kApproachPick at {}",
//                           env_state.get_iiwa_time());
//      }
//
//      if (iiwa_move_.ActionFinished(env_state)) {
//        state_ = kGrasp;
//        iiwa_callback(&stopped_plan);
//        iiwa_move_.Reset();
//      }
//      break;
//    }
//
//    case kGrasp: {
//      // Grasps the object.
//      if (!wsg_act_.ActionStarted()) {
//        lcmt_schunk_wsg_command msg;
//        wsg_act_.CloseGripper(env_state, &msg);
//        wsg_callback(&msg);
//
//        drake::log()->info("kGrasp at {}", env_state.get_iiwa_time());
//      }
//
//      if (wsg_act_.ActionFinished(env_state)) {
//        state_ = kLiftFromPick;
//        wsg_act_.Reset();
//      }
//      break;
//    }
//
//    case kLiftFromPick: {
//      // Lifts the object straight up.
//      if (!iiwa_move_.ActionStarted()) {
//        X_Wend_effector_0_ = X_Wend_effector_1_;
//        X_Wend_effector_1_.translation()[2] += kPreGraspHeightOffset;
//
//        // 2 seconds, 3 via points.
//        bool res = PlanStraightLineMotion(
//            env_state.get_iiwa_q(), 3, 2,
//            X_Wend_effector_0_, X_Wend_effector_1_,
//            tight_pos_tol_, tight_rot_tol_, planner, &ik_res, &times);
//        DRAKE_DEMAND(res);
//
//        robotlocomotion::robot_plan_t plan{};
//        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
//        iiwa_callback(&plan);
//
//        drake::log()->info("kLiftFromPick at {}", env_state.get_iiwa_time());
//      }
//
//      if (iiwa_move_.ActionFinished(env_state)) {
//        state_ = kApproachPlacePregrasp;
//        iiwa_move_.Reset();
//      }
//      break;
//    }
//
//    case kApproachPlacePregrasp: {
//      // Uses 2 seconds to move to right about the target place location.
//      if (!iiwa_move_.ActionStarted()) {
//        X_IIWAobj_desired_ = place_locations_[next_place_location_];
//        const Isometry3<double>& iiwa_base = env_state.get_iiwa_base();
//        X_Wobj_desired_ = iiwa_base * X_IIWAobj_desired_;
//
//        X_Wend_effector_0_ = X_Wend_effector_1_;
//        X_Wend_effector_1_ = ComputeGraspPose(X_Wobj_desired_);
//        X_Wend_effector_1_.translation()[2] += kPreGraspHeightOffset;
//
//        // 2 seconds, no via points.
//        bool res = PlanStraightLineMotion(
//            env_state.get_iiwa_q(), 0, 2,
//            X_Wend_effector_0_, X_Wend_effector_1_,
//            loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
//        DRAKE_DEMAND(res);
//
//        robotlocomotion::robot_plan_t plan{};
//        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
//        iiwa_callback(&plan);
//
//        drake::log()->info("kApproachPlacePregrasp at {}",
//                           env_state.get_iiwa_time());
//      }
//
//      if (iiwa_move_.ActionFinished(env_state)) {
//        state_ = kApproachPlace;
//        iiwa_move_.Reset();
//      }
//      break;
//    }
//
//    case kApproachPlace: {
//      // Moves straight down.
//      if (!iiwa_move_.ActionStarted()) {
//        // Computes the desired end effector pose in the world frame.
//        X_Wend_effector_0_ = X_Wend_effector_1_;
//        X_Wend_effector_1_ = ComputeGraspPose(X_Wobj_desired_);
//
//        // 2 seconds, 3 via points.
//        bool res = PlanStraightLineMotion(
//            env_state.get_iiwa_q(), 3, 2,
//            X_Wend_effector_0_, X_Wend_effector_1_,
//            tight_pos_tol_, tight_rot_tol_, planner, &ik_res, &times);
//        DRAKE_DEMAND(res);
//
//        robotlocomotion::robot_plan_t plan{};
//        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
//        iiwa_callback(&plan);
//
//        drake::log()->info("kApproachPlace at {}", env_state.get_iiwa_time());
//      }
//
//      if (iiwa_move_.ActionFinished(env_state)) {
//        state_ = kPlace;
//        iiwa_callback(&stopped_plan);
//        iiwa_move_.Reset();
//      }
//      break;
//    }
//
//    case kPlace: {
//      // Releases the object.
//      if (!wsg_act_.ActionStarted()) {
//        lcmt_schunk_wsg_command msg;
//        wsg_act_.OpenGripper(env_state, &msg);
//        wsg_callback(&msg);
//
//        drake::log()->info("kPlace at {}", env_state.get_iiwa_time());
//      }
//
//      if (wsg_act_.ActionFinished(env_state)) {
//        state_ = kLiftFromPlace;
//        wsg_act_.Reset();
//      }
//      break;
//    }
//
//    case kLiftFromPlace: {
//      // Moves straight up.
//      if (!iiwa_move_.ActionStarted()) {
//        X_Wend_effector_0_ = X_Wend_effector_1_;
//        X_Wend_effector_1_.translation()[2] += kPreGraspHeightOffset;
//
//        // 2 seconds, 5 via points.
//        bool res = PlanStraightLineMotion(
//            env_state.get_iiwa_q(), 5, 2,
//            X_Wend_effector_0_, X_Wend_effector_1_,
//            tight_pos_tol_, tight_rot_tol_, planner, &ik_res, &times);
//        DRAKE_DEMAND(res);
//
//        robotlocomotion::robot_plan_t plan{};
//        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
//        iiwa_callback(&plan);
//
//        drake::log()->info("kLiftFromPlace at {}", env_state.get_iiwa_time());
//      }
//
//      if (iiwa_move_.ActionFinished(env_state)) {
//        next_place_location_++;
//        if (next_place_location_ == static_cast<int>(place_locations_.size()) &&
//            !loop_) {
//          state_ = kDone;
//          iiwa_callback(&stopped_plan);
//          drake::log()->info("kDone at {}", env_state.get_iiwa_time());
//        } else {
//          next_place_location_ %= place_locations_.size();
//          state_ = kOpenGripper;
//        }
//        iiwa_move_.Reset();
//      }
//      break;
//    }

    case kDone: {
      break;
    }
  }
}

}  // namespace push_and_pick
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

#include "drake/examples/kuka_iiwa_arm/dev/push_pick_place/push_and_pick_state_machine.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
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
const double kPrePushHeightOffset = 0.18;
const double kPenetrationDepth = 0.08;
using pick_and_place::PlanStraightLineMotion;
using pick_and_place::ComputeGraspPose;
//const Vector3<double> kDesiredGraspPosition(0.228, -0.243, 0.7545);
const Vector3<double> kDesiredSidewaysGraspPosition(0.228, -0.27, 0.77045);
const Vector3<double> kDesiredGraspPosition(0.228, -0.243, 0.77045);
const Vector3<double> kDesiredGraspGripperPosition(0.228 + 0.23/2, -0.356, 0.77045);
const Vector3<double> kLiftFromPickPosition(0.0, -0.625087, 1.7645);
const Matrix3<double> kDesiredGraspGripperOrientation(
    (Eigen::MatrixXd(3,3) <<
                       0, 0, 1,
                       0, 1, 0,
                      -1, 0, 0).finished());
const Matrix3<double> kDesiredGraspObjectOrientation(
    (Matrix3<double>::Identity()));
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

        Eigen::AngleAxisd object_pose(env_state.get_object_pose().linear());
        drake::log()->info("Axis angle of object : {}, {}",
                           object_pose.axis().transpose(),
                           object_pose.angle());

        //if(env_state.get_object_pose().linear())
        state_ = kApproachPreSidewaysYPush;
            //kApproachPreSidewaysPick;
            //kApproachPreSidewaysYPush;
            //kApproachPrePushRotate;
        wsg_act_.Reset();
      }
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
}

}  // namespace push_and_pick
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

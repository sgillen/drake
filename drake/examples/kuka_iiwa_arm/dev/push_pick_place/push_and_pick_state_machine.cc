#include "drake/examples/kuka_iiwa_arm/dev/push_pick_place/push_and_pick_state_machine.h"

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace push_and_pick {
namespace {
using pick_and_place::WorldState;
using pick_and_place::IiwaMove;
using pick_and_place::WsgAction;

using manipulation::planner::ConstraintRelaxingIk;

// Position the gripper 30cm above the object before grasp.
const double kPreGraspHeightOffset = 0.3;
//
//// Computes the desired end effector pose in the world frame given the object
//// pose in the world frame.
//Isometry3<double> ComputeGraspPose(const Isometry3<double>& X_WObj) {
//  // Sets desired end effector location to be 12cm behind the object,
//  // with the same orientation relative to the object frame. This number
//  // dependents on the length of the finger and how the gripper is attached.
//  const double kEndEffectorToMidFingerDepth = 0.12;
//  Isometry3<double> X_ObjEndEffector_desired;
//  X_ObjEndEffector_desired.translation() =
//      Vector3<double>(-kEndEffectorToMidFingerDepth, 0, 0);
//  X_ObjEndEffector_desired.linear().setIdentity();
//  return X_WObj * X_ObjEndEffector_desired;
//}

// Computes the desired end effector pose in the world frame given the object
// pose in the world frame.
Isometry3<double> ComputePushPose(const Isometry3<double>& X_WObj) {
  // Sets desired end effector location to be 12cm behind the object,
  // with the same orientation relative to the object frame. This number
  // dependents on the length of the finger and how the gripper is attached.

  // ALso adding a rotation of PI about X in order to be facing down at the
  // object.

  drake::log()->info("Object position : {}", X_WObj.translation().transpose());
  Eigen::AngleAxis<double> temp{Eigen::Quaternion<double>(X_WObj.linear())};
  drake::log()->info("Object pose : ang {}, ax {}", temp.angle(), temp.axis().transpose());
  //const double kEndEffectorToMidFingerDepth = 0.12;
  Isometry3<double> X_ObjEndEffector_desired;
  X_ObjEndEffector_desired.translation() =
      Vector3<double>(0, 0, 0);
  X_ObjEndEffector_desired.linear() =
      Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY()) * Eigen::Matrix3d::Identity();

  Eigen::Isometry3d return_val =  X_WObj * X_ObjEndEffector_desired;

  drake::log()->info("Grasp position : {}", return_val.translation().transpose());
  Eigen::AngleAxis<double> temp2{Eigen::Quaternion<double>(return_val.linear())};
  drake::log()->info("Grasp pose : ang {}, ax {}", temp2.angle(), temp2.axis().transpose());
  return return_val;

}


// Generates a sequence (@p num_via_points + 1) of key frames s.t. the end
// effector moves in a straight line between @pX_WEndEffector0 and
// @p X_WEndEffector1. Orientation is interpolated with slerp. Intermediate
// waypoints' tolerance can be adjusted separately.
bool PlanStraightLineMotion(const VectorX<double>& q_current,
                            const int num_via_points, double duration,
                            const Isometry3<double>& X_WEndEffector0,
                            const Isometry3<double>& X_WEndEffector1,
                            const Vector3<double>& via_points_pos_tolerance,
                            const double via_points_rot_tolerance,
                            ConstraintRelaxingIk* planner, IKResults* ik_res,
                            std::vector<double>* times) {
  DRAKE_DEMAND(duration > 0 && num_via_points >= 0);
  // Makes a slerp trajectory from start to end.
  const eigen_aligned_std_vector<Quaternion<double>> quats = {
      Quaternion<double>(X_WEndEffector0.linear()),
      Quaternion<double>(X_WEndEffector1.linear())};

  const std::vector<MatrixX<double>> pos = {X_WEndEffector0.translation(),
                                            X_WEndEffector1.translation()};
  drake::log()->debug(
      "Planning straight line from {} {} to {} {}",
      pos[0].transpose(), math::rotmat2rpy(X_WEndEffector0.rotation()),
      pos[1].transpose(), math::rotmat2rpy(X_WEndEffector1.rotation()));

  PiecewiseQuaternionSlerp<double> rot_traj({0, duration}, quats);
  PiecewisePolynomial<double> pos_traj =
      PiecewisePolynomial<double>::FirstOrderHold({0, duration}, pos);

  std::vector<
      ConstraintRelaxingIk::IkCartesianWaypoint> waypoints(num_via_points + 1);
  const double dt = duration / (num_via_points + 1);
  double time = 0;
  times->clear();
  times->push_back(time);
  for (int i = 0; i <= num_via_points; ++i) {
    time += dt;
    times->push_back(time);
    waypoints[i].pose.translation() = pos_traj.value(time);
    waypoints[i].pose.linear() = Matrix3<double>(rot_traj.orientation(time));
    drake::log()->debug(
        "via ({}/{}): {} {}", i, num_via_points,
        waypoints[i].pose.translation().transpose(),
        math::rotmat2rpy(waypoints[i].pose.rotation()).transpose());
    if (i != num_via_points) {
      waypoints[i].pos_tol = via_points_pos_tolerance;
      waypoints[i].rot_tol = via_points_rot_tolerance;
    }
    waypoints[i].constrain_orientation = true;
  }
  DRAKE_DEMAND(times->size() == waypoints.size() + 1);
  const bool planner_result =
      planner->PlanSequentialTrajectory(waypoints, q_current, ik_res);
  drake::log()->debug("q initial: {}", q_current.transpose());
  if (!ik_res->q_sol.empty()) {
    drake::log()->debug("q final: {}", ik_res->q_sol.back().transpose());
  }
  drake::log()->debug("result: {}", planner_result);
  return planner_result;
}

}  // namespace

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

        const Isometry3<double>& iiwa_base = env_state.get_iiwa_base();

        drake::log()->info("IIWA base position {}", iiwa_base.translation().transpose());
        drake::log()->info("IIWA base position ionv {}", iiwa_base.inverse().translation().transpose());
        drake::log()->info("object position {}", env_state.get_object_pose().translation().transpose());
        drake::log()->info("object position rel IIWA", (iiwa_base.inverse() * env_state.get_object_pose()).translation().transpose());

         // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
        X_Wend_effector_1_ = ComputePushPose(env_state.get_object_pose());
       // drake::log()->info("translation target re offset = {}", X_Wend_effector_1_.translation().transpose());
        X_Wend_effector_1_.translation()[2] += kPreGraspHeightOffset;
        X_Wend_effector_1_.translation()[1] -= 0.0;
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
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kDone;//kApproachPick;
        iiwa_move_.Reset();
      }
      break;
    }

    case kApproachVerticalPush : break;
    case kVerticalPushMove : break;
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

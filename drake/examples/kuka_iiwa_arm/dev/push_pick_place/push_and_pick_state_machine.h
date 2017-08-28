#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/systems/framework/leaf_system.h"

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/action.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/world_state.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace push_and_pick {
using pick_and_place::WorldState;
using pick_and_place::IiwaMove;
using pick_and_place::WsgAction;
//
///// Different states for the push and pick task.
//enum VerticalPushAndPickState {
//  kOpenGripper,
//  kApproachPreVerticalPush,
//  kApproachVerticalPush,
//  kVerticalPushMove,
////  kApproachPreSidewaysPush,
////  kApproachSidewaysPush,
////  kSidewaysPushMove,
////  kLiftFromVerticalPush
////  kApproachPreReorientPush,
////  kApproachReorientPush,
////  kReorientPushMove,
////  kApproachPreSidewaysPick,
////  kApproachSidewaysPick,
////  kCloseGripper,
////  kLiftFromPick,
////  kReturnToHome,
////  kOpenGripper
//  kDone
//};


/// Different states for the push and pick task.
enum PushAndPickState {
  kOpenGripper,
  kApproachPrePushRotate,
  kApproachPushRotate,
  kPushRotate,
  kApproachPreSidewaysXPush,
  kApproachSidewaysXPush,
  kSidewaysXPushRotate,
  kSidewaysYPushRotate,
  kSidewaysXPushMove,
  kApproachPreSidewaysYPush,
  kApproachSidewaysYPush,
  kSidewaysYPushMove,
  kApproachPreSidewaysPick,
  kApproachSidewaysPick,
  kCloseGripper,
  kLiftFromPick,
//  kReturnToHome,
//  kOpenGripper
      kDone
};

/// A class which controls the pick and place actions for moving a
/// single target in the environment.
class PushAndPickStateMachine {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PushAndPickStateMachine)

  typedef std::function<void(
      const robotlocomotion::robot_plan_t*)> IiwaPublishCallback;
  typedef std::function<void(
      const lcmt_schunk_wsg_command*)> WsgPublishCallback;

  /// Construct a push and pick state machine.  @p place_locations
  /// should contain a list of locations to place the target.  The
  /// state machine will cycle through the target locations, placing
  /// the item then picking it back up at the next target.  If @p loop
  /// is true, the state machine will loop through the pick and place
  /// locations, otherwise it will remain in the kDone state once
  /// complete.
  PushAndPickStateMachine(bool loop);
  ~PushAndPickStateMachine();

  /// Update the state machine based on the state of the world in @p
  /// env_state.  When a new robot plan is available, @p iiwa_callback
  /// will be invoked with the new plan.  If the desired gripper state
  /// changes, @p wsg_callback is invoked.  @p planner should contain
  /// an appropriate planner for the robot.
  void Update(const WorldState& env_state,
              const IiwaPublishCallback& iiwa_callback,
              const WsgPublishCallback& wsg_callback,
              manipulation::planner::ConstraintRelaxingIk* planner);


  PushAndPickState state() const { return state_; }

 private:
  std::vector<Isometry3<double>> place_locations_;
  int next_place_location_;
  bool loop_;

  WsgAction wsg_act_;
  IiwaMove iiwa_move_;

  PushAndPickState state_;

  // Poses used for storing end-points of Iiwa trajectories at various states
  // of the demo.
  Isometry3<double> X_Wend_effector_0_;
  Isometry3<double> X_Wend_effector_1_;

  // Desired object end pose relative to the base of the iiwa arm.
  Isometry3<double> X_IIWAobj_desired_;

  // Desired object end pose in the world frame.
  Isometry3<double> X_Wobj_desired_;

  Vector3<double> tight_pos_tol_;
  double tight_rot_tol_;

  Vector3<double> loose_pos_tol_;
  double loose_rot_tol_;
};

}  // namespace push_and_pick
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

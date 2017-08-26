#pragma once

#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/math/rotation_matrix.h"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace push_and_pick {

//// Computes the desired end effector pose in the world frame given the object
//// pose in the world frame.
Isometry3<double> ComputeSidewaysGraspPose(const Isometry3<double>& X_WObj) {
  // Sets desired end effector location to be 12cm behind the object,
  // with the same orientation relative to the object frame. This number
  // dependents on the length of the finger and how the gripper is attached.
  const double kEndEffectorToMidFingerDepth = 0.12;
  Isometry3<double> X_ObjEndEffector_desired;
  X_ObjEndEffector_desired.translation() =
      Vector3<double>(-kEndEffectorToMidFingerDepth, 0, 0);
  X_ObjEndEffector_desired.linear().setIdentity();
  return X_WObj * X_ObjEndEffector_desired;
}

// Computes the desired end effector pose in the world frame given the object
// pose in the world frame.
Isometry3<double> ComputeTopDownPushPose(
    const Isometry3<double> &X_WObj, double penetration_depth) {
  // Sets desired end effector location to be 12cm behind the object,
  // with the same orientation relative to the object frame. This number
  // dependents on the length of the finger and how the gripper is attached.

  // Also adding a rotation of PI about X in order to be facing down at the
  // object.

  Isometry3<double> X_OE_desired;
  X_OE_desired.translation() =
      Vector3<double>(0, 0, 0.01 /* book thickness */ + penetration_depth);
  X_OE_desired.linear() =
      Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(0.0 * M_PI, Eigen::Vector3d::UnitZ()) *
          Eigen::Matrix3d::Identity();

  Eigen::Isometry3d return_val =  X_WObj * X_OE_desired;

  //return_val.translation() = X_WE_desired.translation();

  drake::log()->info("Grasp position : {}", return_val.translation().transpose());
  Eigen::AngleAxis<double> temp2{Eigen::Quaternion<double>(return_val.linear())};
  drake::log()->info("Grasp pose : ang {}, ax {}", temp2.angle(), temp2.axis().transpose());
  return return_val;

}

// Computes the desired end effector pose in the world frame given the object
// pose in the world frame.
Isometry3<double> ComputeSidewaysPushPose(const Isometry3<double> &X_WObj) {
  // Sets desired end effector location to be 12cm behind the object,
  // with the same orientation relative to the object frame. This number
  // dependents on the length of the finger and how the gripper is attached.

  // Also adding a rotation of PI about X in order to be facing down at the
  // object.

  const double kPenetrationDepth = 0.14; //0.0* 0.001;
  Isometry3<double> X_OE_desired;
  X_OE_desired.translation() =
      Vector3<double>(0, -0.09, 0.01 /* book thickness */ + kPenetrationDepth);
  X_OE_desired.linear() =
      Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(0.0 * M_PI, Eigen::Vector3d::UnitZ()) *
          Eigen::Matrix3d::Identity();

  Eigen::Isometry3d return_val =  X_WObj * X_OE_desired;

  //return_val.translation() = X_WE_desired.translation();

  drake::log()->info("Grasp position : {}", return_val.translation().transpose());
  Eigen::AngleAxis<double> temp2{Eigen::Quaternion<double>(return_val.linear())};
  drake::log()->info("Grasp pose : ang {}, ax {}", temp2.angle(), temp2.axis().transpose());
  return return_val;

}


}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

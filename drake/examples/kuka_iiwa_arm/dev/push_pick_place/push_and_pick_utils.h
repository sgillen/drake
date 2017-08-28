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
Isometry3<double> ComputeSidewaysGraspPose(const Isometry3<double>& X_WObj, double displacement, double angle) {
  // Sets desired end effector location to be 12cm behind the object,
  // with the same orientation relative to the object frame. This number
  // dependents on the length of the finger and how the gripper is attached.
  Isometry3<double> X_OE_desired;
  X_OE_desired.translation() =
      Vector3<double>(0, displacement, 0);
  X_OE_desired.linear() =
      Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) *
          Eigen::Matrix3d::Identity();
  Eigen::Isometry3d return_val =  X_WObj * X_OE_desired;

  drake::log()->info("Grasp pose : \n{}", return_val.linear());

  return return_val;
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
          Eigen::AngleAxisd(0.0 * M_PI, Eigen::Vector3d::UnitX()) *
          Eigen::Matrix3d::Identity();

  Eigen::Isometry3d return_val =  X_WObj * X_OE_desired;

  //return_val.translation() = X_WE_desired.translation();
//
//  drake::log()->info("Grasp position : {}", return_val.translation().transpose());
//  Eigen::AngleAxis<double> temp2{Eigen::Quaternion<double>(return_val.linear())};
//  drake::log()->info("Grasp pose : ang {}, ax {}", temp2.angle(), temp2.axis().transpose());
  return return_val;

}

// Computes the desired end effector pose in the world frame given the object
// pose in the world frame.
Isometry3<double> ComputeEdgeXPushPose(const Isometry3<double> &X_WObj, double penetration_depth) {
// Sets desired end effector location to be 12cm behind the object,
// with the same orientation relative to the object frame. This number
// dependents on the length of the finger and how the gripper is attached.

// Also adding a rotation of PI about X in order to be facing down at the
// object.

Isometry3<double> X_OE_desired;
X_OE_desired.translation() =
Vector3<double>(0, -0.11, 0.01 /* book thickness */ + penetration_depth);
X_OE_desired.linear() =
Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitX()) *
    Eigen::Matrix3d::Identity();

Eigen::Isometry3d return_val =  X_WObj * X_OE_desired;

//return_val.translation() = X_WE_desired.translation();
//
//drake::log()->info("Grasp position : {}", return_val.translation().transpose());
//Eigen::AngleAxis<double> temp2{Eigen::Quaternion<double>(return_val.linear())};
//drake::log()->info("Grasp pose : ang {}, ax {}", temp2.angle(), temp2.axis().transpose());
return return_val;
}

// Computes the desired end effector pose in the world frame given the object
// pose in the world frame.
Isometry3<double> ComputeEdgeYPushPose(const Isometry3<double> &X_WObj, double penetration_depth) {
// Sets desired end effector location to be 12cm behind the object,
// with the same orientation relative to the object frame. This number
// dependents on the length of the finger and how the gripper is attached.

// Also adding a rotation of PI about X in order to be facing down at the
// object.

  Isometry3<double> X_OE_desired;
  X_OE_desired.translation() =
      Vector3<double>(0.135, 0, 0.01 /* book thickness */ + penetration_depth);
  X_OE_desired.linear() =
      Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(0.0 * M_PI, Eigen::Vector3d::UnitX()) *
          Eigen::Matrix3d::Identity();

  Eigen::Isometry3d return_val =  X_WObj * X_OE_desired;

//return_val.translation() = X_WE_desired.translation();
//
//drake::log()->info("Grasp position : {}", return_val.translation().transpose());
//Eigen::AngleAxis<double> temp2{Eigen::Quaternion<double>(return_val.linear())};
//drake::log()->info("Grasp pose : ang {}, ax {}", temp2.angle(), temp2.axis().transpose());
  return return_val;
}


bool IsPositionXYClose(const Vector3<double> &position1, const Vector3<double> &position2, double tolerance) {
  Vector2<double> diff = position1.topRows(2) - position2;
  double diff_val = diff.squaredNorm();
  if( diff_val < tolerance)
    return true;
  else
    return false;
}

bool IsOrientationClose(const Matrix3<double>& orientation1,
                     const Matrix3<double>& orientation2,
                     double orientation_tolerance) {
  Matrix3<double> orientation_diff = orientation1 - orientation2;
  double orientation_abs = orientation_diff.squaredNorm();
  drake::log()->info("COMPARISON orientation1 \n{}, \norientation2 \n{}", orientation1, orientation2);
  drake::log()->info("orientation abs : {}", orientation_abs);
  if(orientation_abs < orientation_tolerance )
    return true;
  else
    return false;
}

bool IsPoseClose(const Isometry3<double>& pose1, const Isometry3<double>& pose2, double position_tolerance, double orientation_tolerance) {
  Vector3<double> position_diff = pose1.translation() - pose2.translation();
  Matrix3<double> orientation_diff = pose1.linear() - pose2.linear();
  double position_abs = position_diff.squaredNorm();
  double orientation_abs = orientation_diff.squaredNorm();

  if(position_abs < position_tolerance && orientation_abs < orientation_tolerance )
    return true;
  else
    return false;
}


}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

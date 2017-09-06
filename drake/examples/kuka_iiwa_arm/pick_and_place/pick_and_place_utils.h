#pragma once

#include <vector>

#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/math/rotation_matrix.h"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

using manipulation::planner::ConstraintRelaxingIk;

// Computes the desired end effector pose in the world frame given the object
// pose in the world frame.
Isometry3<double> ComputeGraspPose(const Isometry3<double>& X_WObj) {
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


bool PlanSequenceMotion(const VectorX<double>& q_current,
                        const int upsample, double duration,
                        const std::vector<Isometry3<double>>& X_WEs,
                        const Vector3<double>& via_points_pos_tolerance,
                        const double via_points_rot_tolerance,
                        ConstraintRelaxingIk* planner, IKResults* ik_res,
                        std::vector<double>* ptimes) {
  int num_coarse = X_WEs.size();
  int num_fine = num_coarse * upsample;
  DRAKE_DEMAND(num_coarse > 1);
  DRAKE_DEMAND(duration > 0);
  DRAKE_DEMAND(ptimes != nullptr);

  // Generate coarse times for piecewise trajectories.
  const double dt_k = duration / (num_coarse - 1); 
  std::vector<double> times_coarse(num_coarse);
  for (int k = 0; k < num_coarse; ++k) {
    times_coarse[k] = k * dt_k;
  }
  std::vector<ConstraintRelaxingIk::IkCartesianWaypoint>
      waypoints(num_fine);

  eigen_aligned_std_vector<Quaternion<double>> quats(num_coarse);
  std::vector<MatrixX<double>> pos(num_coarse);
  for (int k = 0; k < num_coarse; ++k) {
    quats[k] = X_WEs[k].linear();
    pos[k] = X_WEs[k].translation();
  }

  // Makes a slerp trajectory from start to end.
  PiecewiseQuaternionSlerp<double> rot_traj(times_coarse, quats);
  PiecewisePolynomial<double> pos_traj =
      PiecewisePolynomial<double>::FirstOrderHold(times_coarse, pos);

  const double dt = duration / (num_fine - 1);
  double time = 0;
  std::vector<double>& times = *ptimes;
  times.clear();
  times.push_back(time);
  for (int i = 0; i < num_fine; ++i) {
    time += dt;
    times.push_back(time);
    waypoints[i].pose.translation() = pos_traj.value(time);
    waypoints[i].pose.linear() = Matrix3<double>(rot_traj.orientation(time));
    if (i + 1 != num_fine) {
      waypoints[i].pos_tol = via_points_pos_tolerance;
      waypoints[i].rot_tol = via_points_rot_tolerance;
    }
    waypoints[i].constrain_orientation = true;
  }
  // ???
  DRAKE_DEMAND(times.size() == waypoints.size() + 1);
  const bool planner_result =
      planner->PlanSequentialTrajectory(waypoints, q_current, ik_res);
  drake::log()->debug("q initial: {}", q_current.transpose());
  if (!ik_res->q_sol.empty()) {
    drake::log()->debug("q final: {}", ik_res->q_sol.back().transpose());
  }
  drake::log()->debug("result: {}", planner_result);
  return planner_result;
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

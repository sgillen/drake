#include "drake/perception/estimators/dev/icp.h"

#include "drake/math/rotation_matrix.h"

using Eigen::Matrix3Xd;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Isometry3d;

namespace drake {
namespace perception {
namespace estimators {

Eigen::Isometry3d ComputePCATransform(const Matrix3Xd& y_W) {
  // @see http://www.cse.wustl.edu/~taoju/cse554/lectures/lect07_Alignment.pdf
  Vector3d y_mean = y_W.rowwise().mean();
  Matrix3Xd y_center = y_W.colwise() - y_mean;
  auto W_yy = (y_center * y_center.transpose()).eval();
  Eigen::Isometry3d X_WB;
  X_WB.setIdentity();
  Eigen::SelfAdjointEigenSolver<Matrix3d> eig(W_yy);
  Matrix3d R_WB = eig.eigenvectors();
  if (R_WB.determinant() < 0) {
    // Apply same correction as in ProjectMatToRotMat.
    R_WB.col(2) *= -1;
  }
  X_WB.linear() = R_WB;
  X_WB.translation() = y_mean;
  return X_WB;
}

Eigen::Isometry3d ComputeSVDTransform(const Matrix3Xd& p_B,
                                      const Matrix3Xd& y_W) {
  const int num_points = p_B.cols();
  DRAKE_DEMAND(y_W.cols() == num_points);
  // First moment: Take the mean of each point collection.
  Vector3d p_B_mean = p_B.rowwise().mean();
  Vector3d y_W_mean = y_W.rowwise().mean();
  // Compute deviations from mean, using expression templates since we only
  // evaluate this once.
  auto p_B_center = p_B.colwise() - p_B_mean;
  auto y_W_center = y_W.colwise() - y_W_mean;
  // Second moment: Compute covariance.
  Matrix3d W_py = p_B_center * y_W_center.transpose() / num_points;
  // Compute SVD decomposition to reduce to a proper SO(3) basis.
  Eigen::Isometry3d X_BW;
  X_BW.setIdentity();
  X_BW.linear() = math::ProjectMatToRotMat(W_py);
  X_BW.translation() = p_B_mean - X_BW.linear() * y_W_mean;
  return X_BW.inverse();
}

}  // namespace estimators
}  // namespace perception
}  // namespace drake

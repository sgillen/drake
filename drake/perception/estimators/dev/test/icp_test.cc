#include "drake/perception/estimators/dev/icp.h"

#include <gtest/gtest.h>

#include "drake/math/roll_pitch_yaw.h"
#include "drake/perception/estimators/dev/test/test_util.h"

using Eigen::Matrix3Xd;
using Eigen::Vector3d;
using Eigen::Isometry3d;

namespace drake {
namespace perception {
namespace estimators {
namespace {

GTEST_TEST(Icp, PcaAndSvd) {
  const Bounds box(
      Interval(-0.01, 0.01),
      Interval(-0.05, 0.05),
      Interval(-0.2, 0.2));
  const double spacing = 0.01;
  // Generate points in body frame.
  Matrix3Xd points_B = GenerateBoxPointCloud(spacing, box);
  // Expect identity on PCA.
  Isometry3d X_BB = Isometry3d::Identity();
  // Show the points in the world frame with identity transform.
  // Note that `Bi` is the body frame inferred by PCA.
  Isometry3d X_BBi_pca = EstimatePcaBodyPose(points_B);
  const double tol = 1e-5;
  // Since our model is geometrically centered, this should be at or near
  // identity with PCA, with possible permutations on the axis signs.
  EXPECT_TRUE(CompareTransformWithoutAxisSign(X_BB, X_BBi_pca, tol));
  // Transform points.
  Isometry3d X_WB;
  X_WB.setIdentity();
  Vector3d xyz(0.1, 0.2, 0.3);
  Vector3d rpy(kPi / 10, 0, 0);
  X_WB.linear() << drake::math::rpy2rotmat(rpy);
  X_WB.translation() << xyz;
  // Quick meta-test on CompareTransformWithoutAxisSign.
  EXPECT_FALSE(CompareTransformWithoutAxisSign(X_BB, X_WB, tol));
  // Transform.
  Matrix3Xd points_B_W = X_WB * points_B;
  // Compute PCA for body. Note that frame `Bj` is also inferred by PCA, and
  // may not represent the same frame as `Bi` due to axis sign permutations.
  Isometry3d X_WBj_pca = EstimatePcaBodyPose(points_B_W);
  EXPECT_TRUE(CompareTransformWithoutAxisSign(X_WB, X_WBj_pca, tol));
  // Compute SVD for the body pose.
  Isometry3d X_WB_svd = ComputeSvdBodyPose(points_B, points_B_W);
  EXPECT_TRUE(CompareTransforms(X_WB, X_WB_svd, tol));
  SimpleVisualizer vis;
  vis.PublishCloud(points_B, "B");
  vis.PublishCloud(points_B_W, "BW");
  // Compute frame if `Bi` and `Bj` are indeed the same.
  Isometry3d X_WB_pca_check = X_WBj_pca * X_BBi_pca.inverse();
  vis.PublishFrames({{"X_BB", X_BB},
                     {"X_WB", X_WB},
                     {"X_BBi_pca", X_BBi_pca},
                     {"X_WBj_pca", X_WBj_pca},
                     {"X_WB_pca_check", X_WB_pca_check},
                     {"X_WB_svd", X_WB_svd}});
}

}  // namespace
}  // namespace estimators
}  // namespace perception
}  // namespace drake

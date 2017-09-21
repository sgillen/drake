#include <gtest/gtest.h>

#include "drake/systems/framework/leaf_system.h"
#include "drake/perception/dev/point_cloud.h"

#include <vtkCleanPolyData.h>
#include <vtkSmartPointer.h>

namespace drake {

using systems::LeafSystem;

namespace perception {

void DownSample(const PointCloud& in, PointCloud* out, double radius) {
  auto clean = vtkSmartPointer<vtkCleanPolyData>::New();
  clean->SetToleranceIsAbsolute(true);
  clean->SetTolerance(radius);
  vtkPolyData* in_poly = internal::GetVtkView(in);
  vtkPolyData* out_poly = internal::GetVtkView(*out);
  clean->SetInputData(in_poly);
  clean->SetOutput(out_poly);
  clean->Update();
  internal::SyncVtkView(out_poly, out);
}

GTEST_TEST(PointCloudSystemTest, DownsampleTest) {
  const int count = 10;
  PointCloud in(count);
  // Populate point cloud.
  auto xyzs = in.mutable_xyzs();
  xyzs.col(1) << 0, 0, 0;
  xyzs.col(0) << -10, 0, 0;
  xyzs.col(2) << 10, 0, 0;
  // Cluster remaining points around (0, 0, 0)
  srand(1234);
  const float radius = 0.01;
  const float rand_band = radius / sqrt(3);
  for (int i = 3; i < count; ++i) {
    xyzs.col(i) << Eigen::Vector3f::Random(rand_band, rand_band);
  }

  // Downsample.
  PointCloud out(0);
  DownSample(in, &out, radius);
  ASSERT_EQ(3, out.size());

  Eigen::Matrix3Xf xyzs_expected(3, 3);
  xyzs_expected.transpose() <<
    0, 0, 0,
    -10, 0, 0,
    10, 0, 0;
  EXPECT_EQ(xyzs_expected, out.xyzs());
}

//
//// Perform radial downsampling of a point cloud.
//class PointCloudDownsample : public LeafSystem<double> {
// public:
//  explicit PointCloudDownsample(double radius)
//      : radius_(radius) {
//
//  }
//
// private:
//  double radius_{};
//};
//
//GTEST_TEST(PointCloudSystemTest, DownSampleTest) {
//  PointCloudDownsample sys;
//}

};  // namespace perception
}  // namespace drake

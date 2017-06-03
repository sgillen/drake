#include "drake/manipulation/estimators/dev/tree_state_portion.h"

#include <gtest/gtest.h>

#include "drake/common/unused.h"
#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace manipulation {
namespace {

typedef VectorStampedXd Vector;
typedef KinematicStateStampedPortion<double> Portion;
typedef StampedValue<double> SV;
typedef Stamp<> S;

template <typename Derived>
bool all_nan(const Eigen::MatrixBase<Derived>& X) {
  return (X.array() != X.array()).all();
}

using Eigen::VectorXd;
using Eigen::VectorXi;
using std::endl;
using std::cout;

GTEST_TEST(KinematicStatePortionTest, StampedTest) {
  StampedValue<double> value = 5;
  EXPECT_EQ(5., static_cast<double>(value));

  const auto positions = (Vector(4) << 10, 20, 30, 40).finished();
  EXPECT_TRUE(all_nan(extract_stamps(positions)));
  const auto velocities = (Vector(4) << 100, 200, 300, 400).finished();
  EXPECT_TRUE(all_nan(extract_stamps(velocities)));

  const auto indices = (VectorXi(3) << 3, 0, 2).finished();

  // Full portion
  auto x{Portion::Make(positions, velocities)};
  auto xp{Portion(indices)};

  // Read from superset
  const auto sub_pos_expected = (VectorXd(3) << 40, 10, 30).finished();
  const auto sub_vel_expected = (VectorXd(3) << 400, 100, 300).finished();
  xp.ReadFromSuperset(x);
  EXPECT_EQ(sub_pos_expected, extract_values(xp.positions().values()));
  EXPECT_EQ(sub_vel_expected, extract_values(xp.velocities().values()));

  // Read from subset
  xp.positions().values() << SV(45, 1), SV(15, 2), SV(35, 3);
  xp.velocities().values() << SV(450, 1), SV(150, 2), SV(350, 3);
  x.ReadFromSubset(xp);

  const auto pos_expected = (VectorXd(4) << 15, 20, 35, 45).finished();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto vel_expected = (VectorXd(4) << 150, 200, 350, 450).finished();
  EXPECT_EQ(pos_expected, extract_values(x.positions().values()));
  EXPECT_EQ(vel_expected, extract_values(x.velocities().values()));
  const auto stamps_expected = (VectorXd(4) << 2, nan, 3, 1).finished();
  EXPECT_TRUE(CompareMatrices(stamps_expected,
                              extract_stamps(x.positions().values())));
  EXPECT_TRUE(CompareMatrices(stamps_expected,
                              extract_stamps(x.velocities().values())));
}

}  // anonymous namespace
}  // namespace manipulation
}  // namespace drake

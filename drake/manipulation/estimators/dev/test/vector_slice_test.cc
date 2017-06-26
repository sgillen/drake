#include "drake/manipulation/estimators/dev/vector_slice.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace manipulation {
namespace {

// typedef VectorStampedXd Vector;
// typedef KinematicStateStampedPortion<double> Portion;
// typedef StampedValue<double> SV;
// typedef Stamp<> S;

template <typename Derived>
bool all_nan(const Eigen::MatrixBase<Derived>& X) {
  return (X.array() != X.array()).all();
}

using Eigen::ArrayXd;

using RowArrayXd = Eigen::Array<double, 1, Eigen::Dynamic>;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::RowVector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::RowVectorXd;
// using Eigen::VectorXi;
// using std::endl;
// using std::cout;

GTEST_TEST(VectorSliceTest, BasicSlice) {
  auto superset =
      (VectorXd(4) << 1, 2, 3, 4).finished();
  VectorSlice slice({0, 3, 2}, 4);

  const auto subset_expected =
      (VectorXd(3) << 1, 4, 3).finished();
  VectorXd subset(3);
  slice.ReadFromSuperset(superset, subset);
  EXPECT_EQ(subset_expected, subset);

  subset.array() *= (ArrayXd(3) << 1, 2, 3).finished();
  const auto superset_expected =
      (VectorXd(4) << 1, 2, 9, 8).finished();
  slice.WriteToSuperset(subset, superset);
  EXPECT_EQ(superset_expected, superset);
}

GTEST_TEST(VectorSliceTest, MatrixSlice) {
  VectorSlice slice({2, 0}, 3);
  Matrix3d X;
  X <<
     1, 2, 3,
     4, 5, 6,
     7, 8, 9;
  Eigen::Matrix2d X_sub;
  slice.ReadFromSupersetMatrix(X, X_sub);

  Eigen::Matrix2d X_sub_expected;
  X_sub_expected <<
     9, 7,
     3, 1;
  EXPECT_EQ(X_sub_expected, X_sub);
}

GTEST_TEST(VectorSliceTest, Views) {
  Matrix3d X;
  X <<
    1, 2, 3,
    4, 5, 6,
    7, 8, 9;

  auto X_rows(MakeRowView(X));
  X_rows[1] *= 10;
  Matrix3d X_expected;
  X_expected <<
    1, 2, 3,
    40, 50, 60,
    7, 8, 9;
  EXPECT_EQ(X_expected, X);

  auto X_cols(MakeColView(X));
  X_cols[0].array() -= 1;
  X_expected <<
    0, 2, 3,
    39, 50, 60,
    6, 8, 9;
  EXPECT_EQ(X_expected, X);

  // Immutable view
  const auto& X_const = X;
  auto X_const_cols = MakeColView(X_const);
  const Vector3d col_expected(3, 60, 9);
  EXPECT_EQ(col_expected, X_const_cols[2]);

  auto X_const_rows = MakeRowView(X_const);
  const RowVector3d row_expected(39, 50, 60);
  EXPECT_EQ(row_expected, X_const_rows[1]);
}

GTEST_TEST(VectorSliceTest, ViewSlice) {
  MatrixXd J(3, 4);
  J <<
    0, 1, 2, 3,
    4, 5, 6, 7,
    8, 9, 10, 11;
  auto J_cols = MakeColView(J);

  VectorSlice col_slice({3, 1, 0}, J.cols());

  MatrixXd J_slice;
  auto J_slice_cols = MakeColView(J_slice);
  J_slice_cols.resizeLike(J_cols, col_slice.size());

  col_slice.ReadFromSuperset(J_cols, J_slice_cols);
  MatrixXd J_slice_expected(3, 3);
  J_slice_expected <<
    3, 1, 0,
    7, 5, 4,
    11, 9, 8;
  EXPECT_EQ(J_slice_expected, J_slice);

  // This actually multiplies each column.
  // Not intuitive that we use rowwise(), but makes sense for reduction.
  J_slice.array().rowwise() *= (RowArrayXd(3) << 10, 20, 30).finished();
  // Write to the superset, this time using temporary column views.
  col_slice.WriteToSuperset(MakeColView(J_slice), MakeColView(J));
  MatrixXd J_expected(3, 4);
  J_expected <<
    0, 20, 2, 30,
    120, 100, 6, 70,
    240, 180, 10, 110;
  EXPECT_EQ(J_expected, J);
}

GTEST_TEST(VectorSliceTest, IterableMatrix) {
  VectorXd x(5);
  x << 1, 2, 3, 4, 5;

  // Mutable vector.
  int i = 0;
  for (auto&& xi : MakeIterableMatrix(x)) {
    EXPECT_EQ(x(i), xi);
    xi *= 2;
    i += 1;
  }

  // Immutable vector.
  const VectorXd& x_const = x;
  i = 0;
  for (auto&& xi : MakeIterableMatrix(x_const)) {
    EXPECT_EQ(x(i), xi);
    // Will trigger a compilation error as expected.
    // xi *= 2;
    i += 1;
  }

  // Immutable matrix.
  const auto X = (Eigen::MatrixXd(2, 2) << 1, 2, 3, 4).finished();

  i = 2;
  for (auto&& xi : MakeIterableMatrix(X.col(1))) {
    EXPECT_EQ(X(i), xi);
    i += 1;
  }
  EXPECT_THROW(MakeIterableMatrix(X.row(1)), std::runtime_error);

  // This will cause a compilation error.
  // EXPECT_THROW(MakeIterableMatrix(X.block(1, 1, 1, 1)), std::runtime_error);
}

}  // anonymous namespace
}  // namespace manipulation
}  // namespace drake

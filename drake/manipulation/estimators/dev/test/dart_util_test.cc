#include  "drake/manipulation/estimators/dev/dart_util.h"

#include <gtest/gtest.h>

namespace drake {
namespace manipulation {
namespace {

GTEST_TEST(DartUtil, Sample) {
  vector<string> a {"a", "d", "b"};
  vector<string> b {"a", "b", "c"};
  vector<int> a_indices;
  vector<int> b_indices;
  GetCommonIndices(a, b, &a_indices, &b_indices);

  vector<int> a_indices_expected {0, 2};
  vector<int> b_indices_expected {0, 1};
  EXPECT_EQ(a_indices_expected, a_indices);
  EXPECT_EQ(b_indices_expected, b_indices);
}

}  // namespace
}  // namespace manipulation
}  // namespace drake

#include  "drake/manipulation/estimators/dev/dart_util.h"

#include <gtest/gtest.h>

namespace drake {
namespace manipulation {
namespace {

GTEST_TEST(DartUtil, CommonIndices) {
  vector<int> x_indices_expected {0, 2}; // a, b
  vector<int> y_indices_expected {1, 0}; // a, b

  {
    vector<string> x {"a", "d", "b"};
    vector<string> y {"b", "a", "c"};
    vector<int> x_indices;
    vector<int> y_indices;
    GetCommonIndices(x, y, &x_indices, &y_indices);

    EXPECT_EQ(x_indices_expected, x_indices);
    EXPECT_EQ(y_indices_expected, y_indices);

    EXPECT_THROW(GetSubIndices(x, y, &x_indices), std::runtime_error);

    vector<string> y_sub = {"a", "b"};
    vector<int> y_sub_indices;
    GetSubIndices(y_sub, y, &y_sub_indices);
    vector<int> y_sub_indices_expected {1, 0};
    EXPECT_EQ(y_sub_indices_expected, y_sub_indices);
  }

  {
    // Check for symbolic variables
    // Use direct references such that we can still leverage operator==.
    symbolic::Variable a("a"), b("b"), c("c"), d("d");
    OptVars x(3);
    x << a, d, b;
    OptVars y(3);
    y << b, a, c;

    vector<int> x_indices;
    vector<int> y_indices;
    GetCommonIndices(MakeIterableMatrix(x), MakeIterableMatrix(y),
                     &x_indices, &y_indices);

    EXPECT_EQ(x_indices_expected, x_indices);
    EXPECT_EQ(y_indices_expected, y_indices);
  }
}

}  // namespace
}  // namespace manipulation
}  // namespace drake

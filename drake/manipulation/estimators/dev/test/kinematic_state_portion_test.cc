#include "drake/manipulation/estimators/dev/tree_state_portion.h"

#include <gtest/gtest.h>

namespace drake {
namespace manipulation {
namespace {

GTEST_TEST(KinematicStatePortionTest, StampedTest) {
  typedef KinematicStateStampedPortion<double> Type;
  Type x;
}

}  // anonymous namespace
}  // namespace manipulation
}  // namespace drake

#include "drake/solvers/mosek_solver.h"

#include <gtest/gtest.h>

namespace drake {
namespace solvers {
namespace test {

GTEST_TEST(MosekSolverNegativeTest, TestFailure) {
  EXPECT_THROW(MosekSolver::AcquireLicense(), std::runtime_error);
}

}  // namespace test
}  // namespace solvers
}  // namespace drake

#include "drake/common/proto/call_python.h"

#include <cmath>

#include <gtest/gtest.h>

namespace drake {
namespace common {

GTEST_TEST(TestCallPython, DispEigenMatrix) {
  Eigen::Matrix2d m;
  m << 1, 2, 3, 4;
  CallPython("print", m);

  Eigen::Matrix<bool, 2, 2> b;
  b << true, false, true, false;
  CallPython("print", b);
}

// GTEST_TEST(TestCallPython, RemoteVarTest) {
//   auto magic = CallPythonSingleOutput("magic", 5);
//   CallPython("print", magic);
//   CallPython("print", "element(0,0) is ");
//   CallPython("print", magic(0, 0));
//   CallPython("print", "element(2,1) is ");
//   CallPython("print", magic(2, 1));
//   CallPython("print", "elements (0:2) are");
//   CallPython("print", magic(Eigen::Vector2d(0, 2)));
//   CallPython("print", "row 2 is ");
//   CallPython("print", magic(2, ToPythonSlice(":")));
//   CallPython("print", "elements [0, 1, 2, 3, 4] are");
//   CallPython("print", magic(Eigen::VectorXd::LinSpaced(5, 0, 4)));

//   CallPython("print", "row 1 (accessed via logicals) is");
//   CallPython("print",
//              magic(
//                 Eigen::Matrix<bool, 5, 1>(false, true, false, false, false),
//                 ToPythonSlice(":")));

//   CallPython("print", "Second column should now be 1,2,3,4,5: ");
//   auto n = magic(ToPythonSlice(":"), 1) = Eigen::Vector5d(1, 2, 3, 4, 5);
//   CallPython("print", n);
//   CallPython("print", magic);
// }

// GTEST_TEST(TestCallPython, SimplePlot) {
//   int N = 100;

//   Eigen::VectorXd time(N), val(N);
//   for (int i = 0; i < N; i++) {
//     time[i] = 0.01 * i;
//     val[i] = sin(2 * M_PI * time[i]);
//   }

//   CallPython("print", "Plotting a (red) sine wave.");
//   CallPython("figure", 1);
//   auto h = CallPython("plot", time, val);
//   CallPython("show");
// }

// GTEST_TEST(TestCallPython, MeshTest) {
//   const int N = 25, M = 35;
//   Eigen::VectorXd x(N), y(M);
//   Eigen::MatrixXd Z(M, N);
//   for (int i = 0; i < N; i++) {
//     x(i) = -3.0 + 6.0 * i / (N - 1);
//   }
//   for (int i = 0; i < M; i++) {
//     y(i) = -3.0 + 6.0 * i / (M - 1);
//   }
//   for (int i = 0; i < N; i++) {
//     for (int j = 0; j < M; j++) {
//       Z(j, i) = 3 * pow(1 - x(i), 2) * exp(-pow(x(i), 2) - pow(y(j) + 1, 2)) -
//                 10 * (x(i) / 5 - pow(x(i), 3) - pow(y(j), 5)) *
//                     exp(-pow(x(i), 2) - pow(y(j), 2)) -
//                 1.0 / 3.0 * exp(-pow(x(i) + 1, 2) - pow(y(j), 2));
//     }
//   }
//   CallPython("print", "Plotting a simple 3D surface");
//   CallPython("figure", 2);
//   CallPython("surf", x, y, Z);
//   CallPython("show");
// }

}  // namespace common
}  // namespace drake

#include <iostream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

Ref<VectorXd> SegmentOf(VectorXd& x) {
  return x.segment(0, 1);
}

int main() {
  VectorXd x(2);
  x << 1, 2;
  Ref<VectorXd> y = SegmentOf(x);
  y << 3;

  // NOTE: Not using `cout` because of uninitialized memory used in string
  // conversions.
  VectorXd z = x * y;

  return 0;
}

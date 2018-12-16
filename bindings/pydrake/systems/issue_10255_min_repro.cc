#include "drake/systems/framework/leaf_system.h"

int main() {
  using drake::systems::PeriodicEventData;
  using drake::systems::leaf_system_detail::GetNextSampleTime;
  using std::nexttoward;
  using std::nextafter;
  PeriodicEventData attribute;
  attribute.set_period_sec(1.);
  attribute.set_offset_sec(0.);

  const double inf = std::numeric_limits<double>::infinity();
  const double time = nexttoward(0., -inf);
  double next_time = GetNextSampleTime(attribute, time);
  fmt::print("time: {}\nnext_time: {}\n", time, next_time);
  if (next_time != 0.) {
    fmt::print("NOT EQUAL!!!\n");
    return 1;
  }
  return 0;
}

/*
[ Output ]

$ bazel run //bindings/pydrake/systems:issue_10255_min_repro_static
time: -4.94066e-324
next_time: 0

$ bazel run //bindings/pydrake/systems:issue_10255_min_repro_shared
time: -4.94066e-324
next_time: 1
NOT EQUAL!!!

*/

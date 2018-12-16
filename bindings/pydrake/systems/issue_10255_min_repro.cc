// HACK
#ifdef NDEBUG
  #undef NDEBUG
#endif

#include <cassert>
#include <cmath>

#include <iostream>
#include <limits>

// Stolen from `leaf_system`.
template <typename T>
static T GetNextSampleTime(
    const T& period, const T& offset,
    const T& current_time_sec) {
  assert(period > 0);
  assert(offset >= 0);

  // If the first sample time hasn't arrived yet, then that is the next
  // sample time.
  if (current_time_sec < offset) {
    return offset;
  }

  // Compute the index in the sequence of samples for the next time to sample,
  // which should be greater than the present time.
  using std::ceil;
  const T offset_time = current_time_sec - offset;
  const T next_k = ceil(offset_time / period);
  T next_t = offset + next_k * period;
  if (next_t <= current_time_sec) {
    next_t = offset + (next_k + 1) * period;
  }
  assert(next_t > current_time_sec);
  return next_t;
}

int main() {
  const double period_sec = 1.;
  const double offset_sec = 0.;

  const double inf = std::numeric_limits<double>::infinity();
  const double time = nexttoward(0., -inf);
  double next_time = GetNextSampleTime(period_sec, offset_sec, time);
  std::cout << "time: " << time << "\nnext_time: " << next_time << "\n";
  if (next_time != 0.) {
    std::cout << "NOT EQUAL!\n";
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
NOT EQUAL!

*/

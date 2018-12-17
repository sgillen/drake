#include <cmath>

#include <iostream>
#include <limits>

int main() {
  std::cout.precision(std::numeric_limits<double>::max_digits10);
  const double zero = 0.;
  const double inf = std::numeric_limits<double>::infinity();
  const double slightly_less = std::nexttoward(0., -inf);
  const bool is_less = slightly_less < zero;
  std::cout
      << "zero: " << zero << "\n"
      << "slightly_less: " << slightly_less << "\n"
      << "is_less: " << (is_less ? "true" : "false") << "\n";
  if (!is_less) {
    std::cout << "BAD\n";
    return 1;
  }
  return 0;
}

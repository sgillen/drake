#include "zero_order_hold.h"

int main() {
  drake::systems::ZeroOrderHold<double> zoh(0.1, 1);
  std::cout << "Hello" << std::endl;
  return 0;
}

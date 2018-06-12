#include <iostream>
#include <memory>

#include "drake/systems/sensors/rgbd_renderer_ospray.h"

namespace drake {
namespace systems {
namespace sensors {
namespace test {

using Eigen::Isometry3d;

int DoMain() {
  std::unique_ptr<RgbdRendererOSPRay> renderer(new RgbdRendererOSPRay({}));
  std::cerr << "renderer: " << renderer.get() << std::endl;
  return 0;
}

}  // namespace test
}  // namespace sensors
}  // namespace systems
}  // namespace drake

int main(int, char**) {
  return drake::systems::sensors::test::DoMain();
}

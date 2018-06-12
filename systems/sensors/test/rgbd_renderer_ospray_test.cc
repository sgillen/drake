#include <iostream>
#include <memory>

#include <vtkAutoInit.h>
#include <vtkNew.h>
#include <vtkOSPRayPass.h>

#include "drake/multibody/shapes/visual_element.h"

VTK_MODULE_INIT(vtkRenderingOpenGL2)

namespace drake {
namespace systems {
namespace sensors {
namespace test {

void test_ospray() {
  vtkNew<vtkOSPRayPass> ospray;
}

int DoMain() {
  DrakeShapes::Sphere sphere(1.);
  test_ospray();

  return 0;
}

}  // namespace test
}  // namespace sensors
}  // namespace systems
}  // namespace drake

int main(int, char**) {
  return drake::systems::sensors::test::DoMain();
}

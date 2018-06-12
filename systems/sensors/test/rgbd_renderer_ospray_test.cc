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

void test_ospray(const DrakeShapes::VisualElement& visual) {
  vtkNew<vtkOSPRayPass> ospray;
  std::cerr << "ospray: " << &(*ospray) << std::endl;
  const DrakeShapes::Geometry& geometry = visual.getGeometry();
  unused(geometry);
}

int DoMain() {
  DrakeShapes::Sphere sphere(1.);
  Eigen::Vector4d rgba;
  rgba << 1, 0, 0, 1;
  DrakeShapes::VisualElement element(
    sphere, Eigen::Isometry3d::Identity(), rgba);

  test_ospray(element);

  return 0;
}

}  // namespace test
}  // namespace sensors
}  // namespace systems
}  // namespace drake

int main(int, char**) {
  return drake::systems::sensors::test::DoMain();
}

#include "drake/systems/sensors/rgbd_renderer_ospray.h"

#include <array>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <vtkActor.h>
#include <vtkAutoInit.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkImageExport.h>
#include <vtkJPEGReader.h>
#include <vtkLight.h>
#include <vtkNew.h>
#include <vtkOBJReader.h>
#include <vtkOSPRayLightNode.h>
#include <vtkOSPRayMaterialLibrary.h>
#include <vtkOSPRayPass.h>
#include <vtkOSPRayRendererNode.h>
#include <vtkPNGReader.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkShaderProgram.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTexture.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkWindowToImageFilter.h>

#include "drake/common/drake_assert.h"
#include "drake/systems/sensors/depth_shaders.h"
#include "drake/systems/sensors/vtk_util.h"

VTK_MODULE_INIT(vtkRenderingOpenGL2)

// TODO(kunimatsu-tri) Refactor RgbdRenderer with SceneGraph when it's
// ready, so that other VTK dependent sensor simulators can share the world
// without duplicating it.

namespace drake {
namespace systems {
namespace sensors {

class RgbdRendererOSPRay::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl(RgbdRendererOSPRay* parent, const Eigen::Isometry3d& X_WC);
  ~Impl() {}

 private:

  RgbdRendererOSPRay* parent_ = nullptr;
};

RgbdRendererOSPRay::Impl::Impl(RgbdRendererOSPRay* parent,
                            const Eigen::Isometry3d& X_WC)
    : parent_(parent) {
  // OSPRay specific configuration.
  vtkNew<vtkOSPRayPass> ospray;
  unused(ospray);
  unused(parent_);
  exit(0);
}

RgbdRendererOSPRay::RgbdRendererOSPRay(const RenderingConfig& config,
                                 const Eigen::Isometry3d& X_WC)
    : RgbdRenderer(config, X_WC),
      impl_(new RgbdRendererOSPRay::Impl(this, X_WC)) {}

RgbdRendererOSPRay::~RgbdRendererOSPRay() {}

optional<RgbdRenderer::VisualIndex> RgbdRendererOSPRay::ImplRegisterVisual(
    const DrakeShapes::VisualElement& visual, int body_id) {
  const DrakeShapes::Geometry& geometry = visual.getGeometry();
  unused(geometry);
  throw std::runtime_error("Stubbed");
}

void RgbdRendererOSPRay::ImplAddFlatTerrain() {
  throw std::runtime_error("Stubbed");
}

void RgbdRendererOSPRay::ImplUpdateViewpoint(
    const Eigen::Isometry3d& X_WC) const {
  throw std::runtime_error("Stubbed");
}

void RgbdRendererOSPRay::ImplUpdateVisualPose(const Eigen::Isometry3d& X_WV,
                                       int body_id,
                                       VisualIndex visual_id) const {
  throw std::runtime_error("Stubbed");
}

void RgbdRendererOSPRay::ImplRenderColorImage(
    ImageRgba8U* color_image_out) const {
  throw std::runtime_error("Stubbed");
}

void RgbdRendererOSPRay::ImplRenderDepthImage(
    ImageDepth32F* depth_image_out) const {
  throw std::runtime_error("Stubbed");
}

void RgbdRendererOSPRay::ImplRenderLabelImage(
    ImageLabel16I* label_image_out) const {
  throw std::runtime_error("Stubbed");
}

void RgbdRendererOSPRay::SetBackground(const std::string& filepath) {
  throw std::runtime_error("Stubbed");
}
}  // namespace sensors
}  // namespace systems
}  // namespace drake

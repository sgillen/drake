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

namespace {

enum ImageType {
  kColor = 0,
};

const int kNumOutputImage = 1;

struct RenderingPipeline {
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> window;
  vtkNew<vtkWindowToImageFilter> filter;
  vtkNew<vtkImageExport> exporter;
};

struct ModuleInitVtkRenderingOpenGL2 {
  ModuleInitVtkRenderingOpenGL2() {
    VTK_AUTOINIT_CONSTRUCT(vtkRenderingOpenGL2)
  }
};

}  // namespace

class RgbdRendererOSPRay::Impl : private ModuleInitVtkRenderingOpenGL2 {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl(RgbdRendererOSPRay* parent, const Eigen::Isometry3d& X_WC);
  ~Impl() {}

 private:

  RgbdRendererOSPRay* parent_ = nullptr;
  vtkNew<vtkLight> light_;
  vtkNew<vtkOSPRayMaterialLibrary> materials_;
  vtkNew<vtkActor> terrain_actor_;
  // Use ImageType to access to this array. We assume pipelines_'s indices to be
  // 0 for RGB, 1 for depth, and 2 for ground-truth label rendering.
  std::array<std::unique_ptr<RenderingPipeline>,
             kNumOutputImage> pipelines_;
};

RgbdRendererOSPRay::Impl::Impl(RgbdRendererOSPRay* parent,
                            const Eigen::Isometry3d& X_WC)
    : parent_(parent),
      pipelines_{{std::make_unique<RenderingPipeline>()}} {
  auto& cp = pipelines_[ImageType::kColor];
  if (parent_->config().show_window) {
    cp->window->SetWindowName("Color Image");
  } else {
    for (auto& pipeline : pipelines_) {
      pipeline->window->SetOffScreenRendering(1);
    }
  }

  // OSPRay specific configuration.
  vtkNew<vtkOSPRayPass> ospray;
  unused(ospray);
  exit(0);
}

RgbdRendererOSPRay::RgbdRendererOSPRay(const RenderingConfig& config,
                                 const Eigen::Isometry3d& X_WC)
    : RgbdRenderer(config, X_WC),
      impl_(new RgbdRendererOSPRay::Impl(this, X_WC)) {}

RgbdRendererOSPRay::~RgbdRendererOSPRay() {}

optional<RgbdRenderer::VisualIndex> RgbdRendererOSPRay::ImplRegisterVisual(
    const DrakeShapes::VisualElement& visual, int body_id) {
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

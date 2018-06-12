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

using vtk_util::ConvertToVtkTransform;

namespace {

// TODO(kunimatsu-tri) Add support for the arbitrary clipping planes.
const double kClippingPlaneNear = 0.01;
const double kClippingPlaneFar = 100.;

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

void SetModelTransformMatrixToVtkCamera(
    vtkCamera* camera, const vtkSmartPointer<vtkTransform>& X_WC) {
  // vtkCamera contains a transformation as the internal state and
  // ApplyTransform multiplies a given transformation on top of the internal
  // transformation. Thus, resetting 'Set{Position, FocalPoint, ViewUp}' is
  // needed here.
  camera->SetPosition(0., 0., 0.);
  camera->SetFocalPoint(0., 0., 1.);  // Sets z-forward.
  camera->SetViewUp(0., -1, 0.);  // Sets y-down. For the detail, please refer
  // to CameraInfo's document.
  camera->ApplyTransform(X_WC);
}

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
  exit(0);
  cp->renderer->SetPass(ospray);
  vtkOSPRayRendererNode::SetRendererType("pathtracer", cp->renderer);
  vtkOSPRayRendererNode::SetSamplesPerPixel(1, cp->renderer);

  double np[3] = {0, 0, 1};
  double ep[3] = {0, 1, 0};
  vtkOSPRayRendererNode::SetNorthPole(np, cp->renderer);
  vtkOSPRayRendererNode::SetEastPole(ep, cp->renderer);
  vtkOSPRayRendererNode::SetMaterialLibrary(materials_, cp->renderer);

  const vtkSmartPointer<vtkTransform> vtk_X_WC = ConvertToVtkTransform(X_WC);

  for (auto& pipeline : pipelines_) {
    auto camera = pipeline->renderer->GetActiveCamera();
    camera->SetViewAngle(parent_->config().fov_y * 180. / M_PI);
    camera->SetClippingRange(kClippingPlaneNear, kClippingPlaneFar);
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WC);
    pipeline->window->SetSize(parent_->config().width,
                              parent_->config().height);
    pipeline->window->AddRenderer(pipeline->renderer);
    pipeline->filter->SetInput(pipeline->window);
    pipeline->filter->SetScale(1);
    pipeline->filter->ReadFrontBufferOff();
    pipeline->filter->SetInputBufferTypeToRGBA();
    pipeline->filter->Update();
    pipeline->exporter->SetInputData(pipeline->filter->GetOutput());
    pipeline->exporter->ImageLowerLeftOff();
  }

  // TODO(kunimatsu-tri) Add API to handle lighting stuff.
  light_->SetPosition(-2, 0, 10);
  light_->SetFocalPoint(0, 0, 0);
  light_->PositionalOff();
  // OSPRay specific control, radius to get soft shadows.
  vtkOSPRayLightNode::SetRadius(2.0, light_);
  light_->SetTransformMatrix(vtk_X_WC->GetMatrix());

  cp->renderer->AddLight(light_);
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

#ifdef NDEBUG
  // For `assert`
  #undef NDEBUG
#endif

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <vtkActor.h>
#include <vtkAutoInit.h>
#include <vtkCamera.h>
#include <vtkImageExport.h>
#include <vtkLight.h>
#include <vtkNew.h>
#include <vtkOBJReader.h>
#include <vtkOSPRayLightNode.h>
#include <vtkOSPRayMaterialLibrary.h>
#include <vtkOSPRayPass.h>
#include <vtkOSPRayRendererNode.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkWindowToImageFilter.h>

VTK_AUTOINIT_DECLARE(vtkRenderingOpenGL2)

namespace tmp {

class ImageRgba8U {
 public:
  using T = std::uint8_t;
  static constexpr int kNumChannels = 4;
  static constexpr int kPixelSize = kNumChannels * sizeof(T);

  ImageRgba8U(int width, int height, T initial_value = {})
      : width_(width), height_(height),
        data_(width * height * kNumChannels, initial_value) {}
  
  T* at(int x, int y) {
    return data_.data() + (x + y * width_) * kNumChannels;
  }

 private:
  int width_;
  int height_;
  std::vector<T> data_;
};

const int kWidth = 640;
const int kHeight = 480;
const double kFovY = M_PI / 4;
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

using ActorCollection = std::vector<vtkSmartPointer<vtkActor>>;

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

template <typename T>
std::unique_ptr<T> CreatePtr(T* value) {
  return std::unique_ptr<T>(value);
}

class RgbdRendererOSPRay : private ModuleInitVtkRenderingOpenGL2 {
 public:
  RgbdRendererOSPRay(
      int width, int height,
      vtkSmartPointer<vtkTransform> X_WC) 
    : width_{width}, height_{height},
      pipelines_{CreatePtr(new RenderingPipeline())} {
    auto& cp = pipelines_[ImageType::kColor];
    constexpr bool show_window = false;
    if (show_window) {
      cp->window->SetWindowName("Color Image");
    } else {
      for (auto& pipeline : pipelines_) {
        pipeline->window->SetOffScreenRendering(1);
      }
    }

    // OSPRay specific configuration.
    vtkNew<vtkOSPRayPass> ospray;
    cp->renderer->SetPass(ospray);
    vtkOSPRayRendererNode::SetRendererType("pathtracer", cp->renderer);
    vtkOSPRayRendererNode::SetSamplesPerPixel(1, cp->renderer);

    double np[3] = {0, 0, 1};
    double ep[3] = {0, 1, 0};
    vtkOSPRayRendererNode::SetNorthPole(np, cp->renderer);
    vtkOSPRayRendererNode::SetEastPole(ep, cp->renderer);
    vtkOSPRayRendererNode::SetMaterialLibrary(materials_, cp->renderer);
    vtkOSPRayRendererNode::SetMaxFrames(10, cp->renderer);

    const vtkSmartPointer<vtkTransform> vtk_X_WC = X_WC;

    for (auto& pipeline : pipelines_) {
      auto camera = pipeline->renderer->GetActiveCamera();
      camera->SetViewAngle(kFovY * 180. / M_PI);
      camera->SetClippingRange(kClippingPlaneNear, kClippingPlaneFar);
      SetModelTransformMatrixToVtkCamera(camera, vtk_X_WC);
      pipeline->window->SetSize(width_, height_);
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

  void RenderColorImage(ImageRgba8U* color_image_out) const {
    // TODO(sherm1) Should evaluate VTK cache entry.
    auto& p = pipelines_[ImageType::kColor];
    p->window->Render();
    vtkRenderer* renderer = p->window->GetRenderers()->GetFirstRenderer();
    int max_frames = vtkOSPRayRendererNode::GetMaxFrames(renderer);

    for (int i = 0; i < max_frames; ++i) {
      p->window->Render();
    }
    p->filter->Modified();
    p->filter->Update();
    p->exporter->Update();
    p->exporter->Export(color_image_out->at(0, 0));
  }

  void RegisterStuff() {
    // RegisterVisual
    std::array<vtkNew<vtkActor>, kNumOutputImage> actors;
    std::array<vtkNew<vtkPolyDataMapper>, kNumOutputImage> mappers;
    const char* mesh_filename = "tmp/plastic_mug.obj";

    vtkNew<vtkOBJReader> mesh_reader;
    mesh_reader->SetFileName(mesh_filename);
    mesh_reader->Update();

    // Changing the scale of the loaded mesh.
    const double scale_x = 1;
    const double scale_y = 1;
    const double scale_z = 1;
    vtkNew<vtkTransform> transform;
    transform->Scale(scale_x, scale_y, scale_z);
    vtkNew<vtkTransformPolyDataFilter> transform_filter;
    transform_filter->SetInputConnection(mesh_reader->GetOutputPort());
    transform_filter->SetTransform(transform.GetPointer());
    transform_filter->Update();

    for (auto& mapper : mappers) {
      mapper->SetInputConnection(transform_filter->GetOutputPort());
    }

    // NOTE: Skipping properties in repro.

    auto& color_actor = actors[ImageType::kColor];
    if (color_actor->GetProperty()->GetNumberOfTextures() == 0) {
      // Taken from diffuse color.
      const std::uint8_t color[] = {255, 0, 0};
      color_actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    }

    vtkNew<vtkTransform> vtk_transform;
    vtk_transform->Identity();
    const int body_id = 0;
    auto& actor_collections = id_object_maps_[body_id];
    for (size_t i = 0; i < actors.size(); ++i) {
      actors[i]->SetMapper(mappers[i].GetPointer());
      actors[i]->SetUserTransform(vtk_transform);
      pipelines_[i]->renderer->AddActor(actors[i].GetPointer());
      actor_collections[i].push_back(actors[i].GetPointer());
    }
  }

 private:
  int width_;
  int height_;
  vtkNew<vtkLight> light_;
  vtkNew<vtkOSPRayMaterialLibrary> materials_;
  // Use ImageType to access to this array. We assume pipelines_'s indices to be
  // 0 for RGB, 1 for depth, and 2 for ground-truth label rendering.
  std::array<std::unique_ptr<RenderingPipeline>,
             kNumOutputImage> pipelines_;

  // A map which takes pairs of a body index in RBT and three vectors of
  // vtkSmartPointer to vtkActor for color, depth and label rendering
  // respectively. Each vtkActor corresponds to an visual element specified in
  // SDF / URDF.
  std::map<int, std::array<ActorCollection, kNumOutputImage>> id_object_maps_;
};

void NoBodyTest() {
  vtkNew<vtkTransform> X_WC;
  X_WC->Identity();
  auto renderer = CreatePtr(new RgbdRendererOSPRay(kWidth, kHeight, X_WC));
  ImageRgba8U color(kWidth, kHeight);
  renderer->RenderColorImage(&color);
}

void MeshTest() {
  vtkNew<vtkTransform> X_WC;
  X_WC->Identity();
  auto renderer = CreatePtr(new RgbdRendererOSPRay(kWidth, kHeight, X_WC));
  ImageRgba8U color(kWidth, kHeight);
  renderer->RegisterStuff();
  renderer->RenderColorImage(&color);
}

}  // namespace tmp

int main(int argc, char** argv) {
  assert(argc == 2 && "Supply test");
  std::string arg = argv[1];
  if (arg == "NoBodyTest") {
    tmp::NoBodyTest();
  } else if (arg == "MeshTest") {
    tmp::MeshTest();
  } else {
    assert(false && "Invalid test");
  }
  std::cerr << "Done" << std::endl;
  return 0;
}

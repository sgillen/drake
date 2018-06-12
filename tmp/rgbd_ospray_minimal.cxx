
VTK_AUTOINIT_DECLARE(vtkRenderingOpenGL2)

struct ModuleInitVtkRenderingOpenGL2 {
  ModuleInitVtkRenderingOpenGL2() {
    VTK_AUTOINIT_CONSTRUCT(vtkRenderingOpenGL2)
  }
};


class RgbdRendererOSPRay::Impl : private ModuleInitVtkRenderingOpenGL2 {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl(RgbdRendererOSPRay* parent, const Eigen::Isometry3d& X_WC);
  ~Impl() {}

  void ImplRenderColorImage(ImageRgba8U* color_image_out) const;

 private:
  RgbdRendererOSPRay* parent_ = nullptr;
  vtkNew<vtkLight> light_;
  vtkNew<vtkOSPRayMaterialLibrary> materials_;
  vtkNew<vtkActor> terrain_actor_;
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

void RgbdRendererOSPRay::Impl::ImplRenderColorImage(
    ImageRgba8U* color_image_out) const {
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
  cp->renderer->SetPass(ospray);
  vtkOSPRayRendererNode::SetRendererType("pathtracer", cp->renderer);
  vtkOSPRayRendererNode::SetSamplesPerPixel(1, cp->renderer);

  double np[3] = {0, 0, 1};
  double ep[3] = {0, 1, 0};
  vtkOSPRayRendererNode::SetNorthPole(np, cp->renderer);
  vtkOSPRayRendererNode::SetEastPole(ep, cp->renderer);
  vtkOSPRayRendererNode::SetMaterialLibrary(materials_, cp->renderer);
  vtkOSPRayRendererNode::SetMaxFrames(10, cp->renderer);

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




int DoMain() {
  const Isometry3d X_WC = Eigen::Translation3d(0, 0, 0) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
  ImageRgba8U color_;
  auto renderer_ = std::make_unique<Renderer>(RenderingConfig{}, X_WC);
  renderer_->RenderColorImage(&color_);
  return 0;
}

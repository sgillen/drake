#include "drake/systems/sensors/rgbd_camera.h"

//#define DRAKE_NO_TIMING

#include <array>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkImageCast.h>
#include <vtkImageFlip.h>
#include <vtkNew.h>
#include <vtkOBJReader.h>
#include <vtkPNGReader.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkVersion.h>
#include <vtkWindowToImageFilter.h>

#if VTK_MAJOR_VERSION >= 6
#include <vtkAutoInit.h>
#endif

#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/vtk_util.h"

#if VTK_MAJOR_VERSION >= 6
VTK_AUTOINIT_DECLARE(vtkRenderingOpenGL2)
#endif

// HACK
#include "drake/common/scoped_timer.h"
using timing::Timer;
using timing::TimerWithMessage;

// TODO(kunimatsu-tri) Refactor RenderingWorld out from RgbdCamera,
// so that other vtk dependent sensor simulators can share the RenderingWorld
// without duplicating it.

namespace drake {
namespace systems {
namespace sensors {
namespace {

const int kPortStateInput = 0;
const int kPortColorImage = 0;
const int kPortDepthImage = 1;
const int kPortLabelImage = 2;
const int kPortCameraPose = 3;

// TODO(kunimatsu-tri) Add support for the arbitrary clipping planes and
// background color.
const double kClippingPlaneNear = 0.01;
const double kClippingPlaneFar = 100.;

// TODO(kunimatsu-tri) Add support for the arbitrary image size and the depth
// ranges.
const int kImageWidth = 640;  // In pixels
const int kImageHeight = 480;  // In pixels
const float kDepthRangeNear = 0.5;
const float kDepthRangeFar = 5.0;
const double kTerrainSize = 100.;

// For Zbuffer value conversion.
const double kA = kClippingPlaneFar / (kClippingPlaneFar - kClippingPlaneNear);
const double kB = -kA * kClippingPlaneNear;

std::string RemoveFileExtension(const std::string& filepath) {
  const size_t last_dot = filepath.find_last_of(".");
  if (last_dot == std::string::npos) {
    DRAKE_DEMAND(false);
  }
  return filepath.substr(0, last_dot);
}

template <typename T>
const std::array<vtkSmartPointer<T>, 3>
MakeVtkInstanceArray(const vtkNew<T>& element1,
                     const vtkNew<T>& element2,
                     const vtkNew<T>& element3) {
  return  std::array<vtkSmartPointer<T>, 3>{{
      vtkSmartPointer<T>(element1.GetPointer()),
      vtkSmartPointer<T>(element2.GetPointer()),
      vtkSmartPointer<T>(element3.GetPointer())}};
}

template <typename T>
const std::array<vtkSmartPointer<T>, 2>
MakeVtkInstanceArray(const vtkNew<T>& element1,
                     const vtkNew<T>& element2) {
  return  std::array<vtkSmartPointer<T>, 2>{{
      vtkSmartPointer<T>(element1.GetPointer()),
      vtkSmartPointer<T>(element2.GetPointer())}};
}

// Defines a color based on its three primary additive colors: red, green, and
// blue. Each of these primary additive colors are in the range of [0, 255].
struct Color {
  int r;  // red
  int g;  // green
  int b;  // blue

  bool operator==(const Color& other) const {
    return this->r == other.r && this->g == other.g && this->b == other.b;
  }
};

// Defines a hash function for unordered_map that takes Color as the key.
struct ColorHash {
  std::size_t operator()(const Color& key) const {
    return (key.r * 256 + key.g) * 256 + key.b;
  }
};

// Defines a color based on its three primary additive colors: red, green, and
// blue. Each of these primary additive colors are in the range of [0, 1].
struct NormalizedColor {
  double r;  // red
  double g;  // green
  double b;  // blue
};

// Creates and holds a palette of colors for visualizing different objects in a
// scene (the intent is for a different color to be applied to each identified
// object). The colors are chosen so as to be easily distinguishable. In other
// words, the intensities are spaced as widely as possible given the number of
// required colors. Black, white and gray, which has the same value for all the
// three color channels, are not part of this color palette. This color palette
// can hold up to 1536 colors.
class ColorPalette {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ColorPalette)
  explicit ColorPalette(int num_colors) {
    const int num = std::ceil(num_colors / 6.);
    DRAKE_DEMAND(num < 256);  // The maximum number of uint8_t.

    for (int i = 0; i < num; ++i) {
      // It is possible to have more colors, but we want the colors to be as
      // distinguishable as possible for visualization purpose.  We can add more
      // colors as needed.
      const int intensity = 255 - i * 255 / num;
      DRAKE_ASSERT(intensity > 0);
      color_palette_.push_back(Color{intensity, 0, 0});
      color_palette_.push_back(Color{0, intensity, 0});
      color_palette_.push_back(Color{0, 0, intensity});
      color_palette_.push_back(Color{intensity, intensity, 0});
      color_palette_.push_back(Color{0, intensity, intensity});
      color_palette_.push_back(Color{intensity, 0, intensity});
    }

    // Creates hash map for ID look up.
    for (size_t i = 0; i < color_palette_.size(); ++i) {
      color_id_map_[color_palette_[i]] = i;
    }
    color_id_map_[kTerrainColor] = RgbdCamera::Label::kFlatTerrain;
    color_id_map_[kSkyColor] = RgbdCamera::Label::kNoBody;
  }

  const Color& get_color(int index) const {
    DRAKE_DEMAND(index < static_cast<int>(color_palette_.size()));
    return color_palette_[index];
  }

  const NormalizedColor get_normalized_color(int index) const {
    NormalizedColor color = Normalize(get_color(index));
    return color;
  }

  const NormalizedColor get_normalized_sky_color() const {
    return Normalize(kSkyColor);
  }

  const NormalizedColor get_normalized_terrain_color() const {
    return Normalize(kTerrainColor);
  }

  int LookUpId(const Color& color) const {
    return color_id_map_.at(color);
  }

 private:
  static NormalizedColor Normalize(const Color& color) {
    NormalizedColor normalized;
    normalized.r = color.r / 255.;
    normalized.g = color.g / 255.;
    normalized.b = color.b / 255.;
    return normalized;
  }

  // These colors are chosen so as to be easily distinguished from the colors in
  // color_palette_. They are guaranteed to be distinct from color_palette_
  // because none of their intensity elements are identical.
  // TODO(kunimatsu-tri) Add support for arbitrary colors for the terrain and
  // the sky.
  const Color kTerrainColor{255, 229, 204};
  const Color kSkyColor{204, 229, 255};
  std::vector<Color> color_palette_;
  std::unordered_map<const Color, int, ColorHash> color_id_map_;
};

// Register the object factories for the vtkRenderingOpenGL2 module.
struct ModuleInitVtkRenderingOpenGL2 {
  ModuleInitVtkRenderingOpenGL2() {
#if VTK_MAJOR_VERSION >= 6
    VTK_AUTOINIT_CONSTRUCT(vtkRenderingOpenGL2)
#endif
  }
};

}  // namespace

void RgbdCamera::ConvertDepthImageToPointCloud(const ImageDepth32F& depth_image,
                                               const CameraInfo& camera_info,
                                               Eigen::Matrix3Xd* point_cloud) {
  if (depth_image.size() != point_cloud->cols()) {
    point_cloud->resize(3, depth_image.size());
  }

  const int height = depth_image.height();
  const int width = depth_image.width();
  const float cx = camera_info.center_x();
  const float cy = camera_info.center_y();
  const float fx_inv = 1.f / camera_info.focal_x();
  const float fy_inv = 1.f / camera_info.focal_y();

  Eigen::Matrix3Xd& pc = *point_cloud;
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      float z = depth_image.at(u, v)[0];
      if (z == InvalidDepth::kTooClose || z == InvalidDepth::kTooFar) {
        pc(0, v * width + u) = InvalidDepth::kTooFar;
        pc(1, v * width + u) = InvalidDepth::kTooFar;
        pc(2, v * width + u) = InvalidDepth::kTooFar;
      } else {
        pc(0, v * width + u) = z * (u - cx) * fx_inv;
        pc(1, v * width + u) = z * (v - cy) * fy_inv;
        pc(2, v * width + u) = z;
      }
    }
  }
}


class RgbdCamera::Impl : private ModuleInitVtkRenderingOpenGL2 {
 public:
  Impl(const RigidBodyTree<double>& tree, const RigidBodyFrame<double>& frame,
       double fov_y, bool show_window, bool fix_camera);

  Impl(const RigidBodyTree<double>& tree, const RigidBodyFrame<double>& frame,
       const Eigen::Vector3d& position, const Eigen::Vector3d& orientation,
       double fov_y, bool show_window, bool fix_camera);

  ~Impl() {}

  static float CheckRangeAndConvertToMeters(float z_buffer_value);

  void set_depth_rel_noise_magnitude(double depth_rel_noise_magnitude) {
    depth_rel_noise_magnitude_ = depth_rel_noise_magnitude;
  }
  void set_is_discrete(bool value) {
    is_discrete_ = value;
  }
  bool is_discrete() const { return is_discrete_; }

  void DoCalcOutput(double t,
                    const Eigen::VectorXd& x,
                    rendering::PoseVector<double>* pcamera_base_pose,
                    ImageBgra8U* pcolor_image, ImageDepth32F* pdepth_image,
                    ImageLabel16I* plabel_image);

  const Eigen::Isometry3d& color_camera_optical_pose() const {
    return X_BC_;
  }

  const Eigen::Isometry3d& depth_camera_optical_pose() const {
    return X_BD_;
  }

  const CameraInfo& color_camera_info() const { return color_camera_info_; }

  const CameraInfo& depth_camera_info() const { return depth_camera_info_; }

  const RigidBodyFrame<double>& frame() const { return frame_; }

  const RigidBodyTree<double>& tree() const { return tree_; }

 private:
  void CreateRenderingWorld();

  void UpdateModelPoses(const KinematicsCache<double>& cache,
                        const Eigen::Isometry3d& X_CW) const;

  void UpdateRenderWindow() const;

  const RigidBodyTree<double>& tree_;
  const RigidBodyFrame<double>& frame_;
  const CameraInfo color_camera_info_;
  const CameraInfo depth_camera_info_;
  const Eigen::Isometry3d X_BC_;
  const Eigen::Isometry3d X_BD_;
  const Eigen::Isometry3d X_WB_initial_;
  const bool kCameraFixed;
  ColorPalette color_palette_;
  vtkNew<vtkActor> terrain_actor_;
  // An array of maps which take pairs of a body index in RBT and a vector of
  // vtkSmartPointer to vtkActor. The each vtkActor corresponds to an visual
  // element specified in SDF / URDF. The first element of this array is for
  // color and depth rendering and the second is for label image rendering.
  std::array<std::map<int, std::vector<vtkSmartPointer<vtkActor>>>, 2>
      id_object_maps_;
  vtkNew<vtkRenderer> color_depth_renderer_;
  vtkNew<vtkRenderer> label_renderer_;
  vtkNew<vtkRenderWindow> color_depth_render_window_;
  vtkNew<vtkRenderWindow> label_render_window_;
  vtkNew<vtkWindowToImageFilter> color_filter_;
  vtkNew<vtkWindowToImageFilter> depth_filter_;
  vtkNew<vtkWindowToImageFilter> label_filter_;

  bool is_discrete_{false};
  double depth_rel_noise_magnitude_{0};
  bool use_openmp_{false};

  // Modelling after: RandomSource
  using Generator = std::mt19937;
  using Distribution = std::normal_distribution<double>;
  Generator noise_generator_;
  Distribution noise_distribution_;
};

RgbdCamera::Impl::Impl(const RigidBodyTree<double>& tree,
                       const RigidBodyFrame<double>& frame,
                       const Eigen::Vector3d& position,
                       const Eigen::Vector3d& orientation,
                       double fov_y, bool show_window, bool fix_camera)
    : tree_(tree), frame_(frame),
      color_camera_info_(kImageWidth, kImageHeight, fov_y),
      depth_camera_info_(kImageWidth, kImageHeight, fov_y),
      // The color sensor's origin (`Co`) is offset by 0.02 m on the Y axis of
      // the RgbdCamera's base coordinate system (`B`).
      // TODO(kunimatsu-tri) Add support for arbitrary relative pose.
      X_BC_(Eigen::Translation3d(0., 0.02, 0.) *
            (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()))),
      // The depth sensor's origin (`Do`) is offset by 0.02 m on Y axis of the
      /// RgbdCamera's base coordinate system (`B`).
      // TODO(kunimatsu-tri) Add support for arbitrary relative pose.
      X_BD_(Eigen::Translation3d(0., 0.02, 0.) *
            (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()))),
      X_WB_initial_(
          Eigen::Translation3d(position[0], position[1], position[2]) *
          Eigen::Isometry3d(math::rpy2rotmat(orientation))),
      kCameraFixed(fix_camera), color_palette_(tree.bodies.size()) {
  if (!show_window) {
    for (auto& window : MakeVtkInstanceArray(color_depth_render_window_,
                                             label_render_window_)) {
      window->SetOffScreenRendering(1);
    }
  }

  CreateRenderingWorld();

  vtkNew<vtkCamera> camera;
  camera->SetPosition(0., 0., 0.);
  camera->SetFocalPoint(0., 0., 1.);  // Sets z-forward.
  camera->SetViewUp(0., -1, 0.);  // Sets y-down.
  camera->SetViewAngle(fov_y * 180. / M_PI);
  camera->SetClippingRange(kClippingPlaneNear, kClippingPlaneFar);

  const auto sky_color = color_palette_.get_normalized_sky_color();
  const auto renderers = MakeVtkInstanceArray<vtkRenderer>(
      color_depth_renderer_, label_renderer_);
  for (auto& renderer : renderers) {
    renderer->SetActiveCamera(camera.GetPointer());
    renderer->SetBackground(sky_color.r, sky_color.g, sky_color.b);
  }

  const auto windows = MakeVtkInstanceArray<vtkRenderWindow>(
      color_depth_render_window_, label_render_window_);
  for (size_t i = 0; i < windows.size(); ++i) {
    windows[i]->SetSize(color_camera_info_.width(),
                        color_camera_info_.height());
    windows[i]->AddRenderer(renderers[i].GetPointer());
  }
  label_render_window_->SetMultiSamples(0);

  color_filter_->SetInput(color_depth_render_window_.GetPointer());
  color_filter_->SetInputBufferTypeToRGBA();
  depth_filter_->SetInput(color_depth_render_window_.GetPointer());
  depth_filter_->SetInputBufferTypeToZBuffer();
  label_filter_->SetInput(label_render_window_.GetPointer());
  label_filter_->SetInputBufferTypeToRGB();

  for (auto& filter : MakeVtkInstanceArray<vtkWindowToImageFilter>(
           color_filter_, depth_filter_, label_filter_)) {
    filter->SetMagnification(1);
    filter->ReadFrontBufferOff();
    filter->Update();
  }
}

RgbdCamera::Impl::Impl(const RigidBodyTree<double>& tree,
                       const RigidBodyFrame<double>& frame,
                       double fov_y, bool show_window, bool fix_camera)
    : Impl::Impl(tree, frame, Eigen::Vector3d(0., 0., 0.),
                 Eigen::Vector3d(0., 0., 0.), fov_y, show_window, fix_camera) {}


void RgbdCamera::Impl::CreateRenderingWorld() {
  auto X_CW = (X_WB_initial_ * X_BC_).inverse();
  for (const auto& body : tree_.bodies) {
    if (body->get_name() == std::string(RigidBodyTreeConstants::kWorldName)) {
      continue;
    }

    if (!body->get_visual_elements().empty()) {
      for (auto& id_object_map : id_object_maps_) {
        std::vector<vtkSmartPointer<vtkActor>> vec;
        id_object_map[body->get_body_index()] = vec;
      }
    }

    for (const auto& visual : body->get_visual_elements()) {
      vtkNew<vtkActor> actor;
      vtkNew<vtkPolyDataMapper> mapper;
      bool shape_matched = true;
      bool texture_found = false;
      const DrakeShapes::Geometry& geometry = visual.getGeometry();
      switch (visual.getShape()) {
        case DrakeShapes::BOX: {
          auto box = dynamic_cast<const DrakeShapes::Box&>(geometry);
          vtkNew<vtkCubeSource> vtk_cube;
          vtk_cube->SetXLength(box.size(0));
          vtk_cube->SetYLength(box.size(1));
          vtk_cube->SetZLength(box.size(2));

          mapper->SetInputConnection(vtk_cube->GetOutputPort());
          break;
        }
        case DrakeShapes::SPHERE: {
          auto sphere = dynamic_cast<const DrakeShapes::Sphere&>(geometry);
          vtkNew<vtkSphereSource> vtk_sphere;
          vtk_sphere->SetRadius(sphere.radius);
          vtk_sphere->SetThetaResolution(50);
          vtk_sphere->SetPhiResolution(50);

          mapper->SetInputConnection(vtk_sphere->GetOutputPort());
          break;
        }
        case DrakeShapes::CYLINDER: {
          auto cylinder = dynamic_cast<const DrakeShapes::Cylinder&>(geometry);
          vtkNew<vtkCylinderSource> vtk_cylinder;
          vtk_cylinder->SetHeight(cylinder.length);
          vtk_cylinder->SetRadius(cylinder.radius);
          vtk_cylinder->SetResolution(50);

          // Since the cylinder in vtkCylinderSource is y-axis aligned, we need
          // to rotate it to be z-axis aligned because that is what Drake uses.
          vtkNew<vtkTransform> transform;
          transform->RotateX(90);
          vtkNew<vtkTransformPolyDataFilter> transform_filter;
          transform_filter->SetInputConnection(vtk_cylinder->GetOutputPort());
          transform_filter->SetTransform(transform.GetPointer());
          transform_filter->Update();

          mapper->SetInputConnection(transform_filter->GetOutputPort());
          break;
        }
        case DrakeShapes::MESH: {
          const auto mesh_filename = dynamic_cast<const DrakeShapes::Mesh&>(
              geometry).resolved_filename_.c_str();

          // TODO(kunimatsu-tri) Add support for other file formats.
          vtkNew<vtkOBJReader> mesh_reader;
          mesh_reader->SetFileName(mesh_filename);
          mesh_reader->Update();

          // TODO(kunimatsu-tri) Guessing the texture file name is bad. Instead,
          // get it from somewhere like `DrakeShapes::MeshWithTexture` when it's
          // implemented.
          // TODO(kunimatsu-tri) Add support for other file formats.
          const std::string texture_file(
              RemoveFileExtension(mesh_filename) + ".png");
          std::ifstream file_exist(texture_file);

          if (file_exist) {
            vtkNew<vtkPNGReader> texture_reader;
            texture_reader->SetFileName(texture_file.c_str());
            texture_reader->Update();

            vtkNew<vtkTexture> texture;
            texture->SetInputConnection(texture_reader->GetOutputPort());
            texture->InterpolateOn();
            actor->SetTexture(texture.GetPointer());
            texture_found = true;
          }

          mapper->SetInputConnection(mesh_reader->GetOutputPort());
          break;
        }
        case DrakeShapes::CAPSULE: {
          // TODO(kunimatsu-tri) Implement this as needed.
          shape_matched = false;
          break;
        }
        default: {
          shape_matched = false;
          break;
        }
      }

      // Registers actors.
      if (shape_matched) {
        if (!texture_found) {
          const auto color = visual.getMaterial();
          actor->GetProperty()->SetColor(color[0], color[1], color[2]);
        }

        const int body_id = body->get_body_index();
        const auto& color = color_palette_.get_normalized_color(body_id);
        vtkNew<vtkActor> actor_for_label;
        actor_for_label->GetProperty()->SetColor(color.r, color.g, color.b);
        // This is to disable shadows and to get an object painted with a single
        // color.
        actor_for_label->GetProperty()->LightingOff();

        // Converts visual's pose in the world to the one in the camera
        // coordinate system.
        const auto X_CVisual = X_CW * visual.getWorldTransform();
        vtkSmartPointer<vtkTransform> vtk_transform =
            VtkUtil::ConvertToVtkTransform(X_CVisual);

        auto renderers = MakeVtkInstanceArray<vtkRenderer>(
            color_depth_renderer_, label_renderer_);
        auto actors = MakeVtkInstanceArray<vtkActor>(actor, actor_for_label);
        for (size_t i = 0; i < actors.size(); ++i) {
          actors[i]->SetMapper(mapper.GetPointer());
          actors[i]->SetUserTransform(vtk_transform);
          renderers[i]->AddActor(actors[i].GetPointer());
          id_object_maps_[i][body_id].push_back(
              vtkSmartPointer<vtkActor>(actors[i].GetPointer()));
        }
      }
    }
  }

  // Adds a flat terrain.
  vtkSmartPointer<vtkPlaneSource> plane = VtkUtil::CreateSquarePlane(
      kTerrainSize);
  vtkSmartPointer<vtkTransform> transform =
      VtkUtil::ConvertToVtkTransform(X_CW);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(plane->GetOutputPort());
  terrain_actor_->SetMapper(mapper.GetPointer());
  auto color = color_palette_.get_normalized_terrain_color();
  terrain_actor_->GetProperty()->SetColor(color.r,
                                          color.g,
                                          color.b);
  terrain_actor_->GetProperty()->LightingOff();
  terrain_actor_->SetUserTransform(transform);
  for (auto& renderer : MakeVtkInstanceArray<vtkRenderer>(color_depth_renderer_,
                                                          label_renderer_)) {
    renderer->AddActor(terrain_actor_.GetPointer());
  }
}

void RgbdCamera::Impl::UpdateModelPoses(
    const KinematicsCache<double>& cache,
    const Eigen::Isometry3d& X_CW) const {
  for (const auto& body : tree_.bodies) {
    if (body->get_name() == std::string(RigidBodyTreeConstants::kWorldName)) {
      continue;
    }

    for (size_t i = 0; i < body->get_visual_elements().size(); ++i) {
      const auto& visual = body->get_visual_elements()[i];
      const auto X_CVisual = X_CW * tree_.CalcBodyPoseInWorldFrame(
          cache, *body) * visual.getLocalTransform();

      vtkSmartPointer<vtkTransform> vtk_transform =
          VtkUtil::ConvertToVtkTransform(X_CVisual);
      // `id_object_maps_` is modified here. This is OK because 1) we are just
      // copying data to the memory spaces allocated at the construction time
      // and 2) we are not outputting these data to outside the class.
      for (auto& id_object_map : id_object_maps_) {
        auto& actor = id_object_map.at(body->get_body_index()).at(i);
        actor->SetUserTransform(vtk_transform);
      }
    }
  }

  if (!kCameraFixed) {
    // Updates terrain.
    vtkSmartPointer<vtkTransform> vtk_transform =
        VtkUtil::ConvertToVtkTransform(X_CW);
    // `terrain_actor_` is modified here, but this is OK.  For the detail, see
    // the comment above for `id_object_maps_`.
    terrain_actor_->SetUserTransform(vtk_transform);
  }
}

void RgbdCamera::Impl::UpdateRenderWindow() const {
//  for (auto& window : MakeVtkInstanceArray<vtkRenderWindow>(
//           color_depth_render_window_, label_render_window_)) {
  {
    SCOPE_TIME(scoped1, "Window");
    color_depth_render_window_->Render();
  }

  for (auto& filter : MakeVtkInstanceArray<vtkWindowToImageFilter>(
           color_filter_, depth_filter_/*, label_filter_*/)) {
    {
      SCOPE_TIME(scoped1, "Filter Modified");
      filter->Modified();
    }
    {
      SCOPE_TIME(scoped1, "Filter Update");
      filter->Update();
    }
  }
}

void RgbdCamera::Impl::DoCalcOutput(
    double t,
    const Eigen::VectorXd& x,
    rendering::PoseVector<double>* camera_base_pose,
    sensors::ImageBgra8U* pcolor_image,
    sensors::ImageDepth32F* pdepth_image,
    sensors::ImageLabel16I* /*plabel_image*/) {
  drake::log()->info("True camera render: {}", t);
  SCOPE_TIME(calc, "DoCalcOutput 1: ");

  static never_destroyed<TimerWithMessage> timer_full("(Across Frame) DoCalc");
  timer_full.access().reset();

  // Dereference
//  auto& camera_base_pose = *pcamera_base_pose;
  auto& color_image = *pcolor_image;
  auto& depth_image = *pdepth_image;
//  auto& label_image = *plabel_image;

  const Eigen::VectorXd q = x.head(tree_.get_num_positions());
  KinematicsCache<double> cache = tree_.doKinematics(q);

  Eigen::Isometry3d X_WB;
  if (kCameraFixed) {
    X_WB = X_WB_initial_;
  } else {
    // Updates camera pose.
    X_WB = tree_.CalcFramePoseInWorldFrame(cache, frame_);
  }

  Eigen::Translation<double, 3> trans = Eigen::Translation<double, 3>(
      X_WB.translation());
  camera_base_pose->set_translation(trans);
  Eigen::Quaterniond quat = Eigen::Quaterniond(X_WB.linear());
  camera_base_pose->set_rotation(quat);

  {
    SCOPE_TIME(scope_timer_2, "UpdateModelPoses");
    UpdateModelPoses(cache, (X_WB * X_BC_).inverse());
  }

  {
    SCOPE_TIME(scope_timer_2, "UpdateRenderWindow");
    UpdateRenderWindow();
  }

  const int height = color_camera_info_.height();
  const int width = color_camera_info_.width();
  SCOPE_TIME(pix, "Pix iter");
  // https://stackoverflow.com/a/39119009/7829525
#pragma omp parallel for collapse(2) if(use_openmp_)
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
//      if (u == 0 && v % 100 == 0) {
//        timer_pix.reset();
//      }
      const int height_reversed = height - v - 1;  // Makes image upside down.

      // We cast `void*` to `uint8_t*` for RGBA, and to `float*` for ZBuffer,
      // respectively. This is because these are the types for pixels internally
      // used in `vtkWindowToImageFiler` class. For more detail, refer to:
      // http://www.vtk.org/doc/release/5.8/html/a02326.html.
      // Converts RGBA to BGRA.
      void* color_ptr = color_filter_->GetOutput()->GetScalarPointer(u, v, 0);
      color_image.at(u, height_reversed)[0] = *(static_cast<uint8_t*>(color_ptr) + 2);
      color_image.at(u, height_reversed)[1] = *(static_cast<uint8_t*>(color_ptr) + 1);
      color_image.at(u, height_reversed)[2] = *(static_cast<uint8_t*>(color_ptr) + 0);
      color_image.at(u, height_reversed)[3] = *(static_cast<uint8_t*>(color_ptr) + 3);

      // Updates the depth image.
      const float z_buffer_value = *static_cast<float*>(
          depth_filter_->GetOutput()->GetScalarPointer(u, v, 0));
      float depth_meters = CheckRangeAndConvertToMeters(z_buffer_value);
      // Add noise
      depth_meters *= (1 + depth_rel_noise_magnitude_ / 3.
                      * noise_distribution_(noise_generator_));
      depth_image.at(u, height_reversed)[0] = depth_meters;

//      // Updates the label image.
//      void* label_ptr = label_filter_->GetOutput()->GetScalarPointer(u, v, 0);
//      Color color{*(static_cast<uint8_t*>(label_ptr) + 0),  // R
//                  *(static_cast<uint8_t*>(label_ptr) + 1),  // G
//                  *(static_cast<uint8_t*>(label_ptr) + 2)};  // B

//      label_image.at(u, height_reversed)[0] =
//          static_cast<int16_t>(color_palette_.LookUpId(color));
    }
  }
}

float RgbdCamera::Impl::CheckRangeAndConvertToMeters(float z_buffer_value) {
  float checked_depth;
  // When the depth is either closer than kClippingPlaneNear or further than
  // kClippingPlaneFar, `z_buffer_value` becomes `1.f`.
  if (z_buffer_value == 1.f) {
    checked_depth = std::numeric_limits<float>::quiet_NaN();
  } else {
    // TODO(kunimatsu-tri) Calculate this in a vertex shader.
    float depth = static_cast<float>(kB / (z_buffer_value - kA));

    if (depth > kDepthRangeFar) {
      checked_depth = InvalidDepth::kTooFar;
    } else if (depth < kDepthRangeNear) {
      checked_depth = InvalidDepth::kTooClose;
    } else {
      checked_depth = depth;
    }
  }

  return checked_depth;
}

RgbdCamera::RgbdCamera(const std::string& name,
                       const RigidBodyTree<double>& tree,
                       const Eigen::Vector3d& position,
                       const Eigen::Vector3d& orientation,
                       double fov_y,
                       bool show_window, double period_sec)
    : impl_(new RgbdCamera::Impl(tree, RigidBodyFrame<double>(), position,
                                 orientation, fov_y, show_window, true)) {
  Init(name, period_sec);
}

RgbdCamera::RgbdCamera(const std::string& name,
                       const RigidBodyTree<double>& tree,
                       const RigidBodyFrame<double>& frame,
                       double fov_y,
                       bool show_window,
                       double period_sec)
    : impl_(new RgbdCamera::Impl(tree, frame, fov_y, show_window, false)) {
  Init(name, period_sec);
}

/**
 * Convenience to infer type.
 */
template <typename T>
std::unique_ptr<T> CreateUnique(T* obj) {
  return std::unique_ptr<T>(obj);
}

void RgbdCamera::Init(const std::string& name, double period_sec) {
  set_name(name);
  const int kVecNum =
      impl_->tree().get_num_positions() + impl_->tree().get_num_velocities();
  this->DeclareInputPort(systems::kVectorValued, kVecNum);

  ImageBgra8U color_image(kImageWidth, kImageHeight);
  this->DeclareAbstractOutputPort(systems::Value<sensors::ImageBgra8U>(
      color_image));

  ImageDepth32F depth_image(kImageWidth, kImageHeight);
  this->DeclareAbstractOutputPort(systems::Value<sensors::ImageDepth32F>(
      depth_image));

  ImageLabel16I label_image(kImageWidth, kImageHeight);
  this->DeclareAbstractOutputPort(systems::Value<sensors::ImageLabel16I>(
      label_image));

  rendering::PoseVector<double> pose_vector;
  this->DeclareVectorOutputPort(pose_vector);

  // TODO(eric.cousineau): Remove this once computation caching is available,
  // such that a ZOH will be able to fully minimize the number of render calls.
  if (period_sec > 0.) {
    using systems::Value;
    // Store values in abstract states
    this->DeclareAbstractState(
        CreateUnique(new Value<sensors::ImageBgra8U>(color_image)));
    this->DeclareAbstractState(
        CreateUnique(new Value<sensors::ImageDepth32F>(depth_image)));
    this->DeclareAbstractState(
        CreateUnique(new Value<sensors::ImageLabel16I>(label_image)));
    // TODO(eric.cousineau): Is there an easier way to allocate discrete states?
    this->DeclareDiscreteState(pose_vector.size());
    this->DeclarePeriodicUnrestrictedUpdate(period_sec, 0.);
    impl_->set_is_discrete(true);
    impl_->set_depth_rel_noise_magnitude(0.02); // 2% noise
    drake::log()->info("WORKAROUND: Using discrete camera update");
  }
}

RgbdCamera::~RgbdCamera() {}

const CameraInfo& RgbdCamera::color_camera_info() const {
  return impl_->color_camera_info();
}

const CameraInfo& RgbdCamera::depth_camera_info() const {
  return impl_->depth_camera_info();
}

const Eigen::Isometry3d& RgbdCamera::color_camera_optical_pose() const {
  return impl_->color_camera_optical_pose();
}

const Eigen::Isometry3d& RgbdCamera::depth_camera_optical_pose() const {
  return impl_->depth_camera_optical_pose();
}

const RigidBodyFrame<double>& RgbdCamera::frame() const {
  return impl_->frame();
}

const RigidBodyTree<double>& RgbdCamera::tree() const {
  return impl_->tree();
}

const InputPortDescriptor<double>& RgbdCamera::state_input_port() const {
  return System<double>::get_input_port(kPortStateInput);
}

const OutputPortDescriptor<double>&
RgbdCamera::color_image_output_port() const {
  return System<double>::get_output_port(kPortColorImage);
}

const OutputPortDescriptor<double>&
RgbdCamera::depth_image_output_port() const {
  return System<double>::get_output_port(kPortDepthImage);
}

const OutputPortDescriptor<double>&
RgbdCamera::label_image_output_port() const {
  return System<double>::get_output_port(kPortLabelImage);
}

const OutputPortDescriptor<double>&
RgbdCamera::camera_base_pose_output_port() const {
  return System<double>::get_output_port(kPortCameraPose);
}

void RgbdCamera::DoCalcOutput(const systems::Context<double>& context,
                              systems::SystemOutput<double>* output) const {
  const Eigen::VectorXd& x =
      this->EvalVectorInput(context, kPortStateInput)->CopyToVector();

  rendering::PoseVector<double>& camera_base_pose =
      *dynamic_cast<rendering::PoseVector<double>*>(
          output->GetMutableVectorData(kPortCameraPose));

  // Outputs the image data.
  sensors::ImageBgra8U& color_image =
      output->GetMutableData(kPortColorImage)->GetMutableValue<
        sensors::ImageBgra8U>();

  sensors::ImageDepth32F& depth_image =
      output->GetMutableData(kPortDepthImage)->GetMutableValue<
        sensors::ImageDepth32F>();

  // sensors::ImageLabel16I& label_image =
  //     output->GetMutableData(kPortLabelImage)->GetMutableValue<
  //       sensors::ImageLabel16I>();

  if (impl_->is_discrete()) {
    // Copy data.
    drake::log()->info("Camera discrete copy: {}", context.get_time());
    const sensors::ImageBgra8U& color_image_state =
        context.get_abstract_state<sensors::ImageBgra8U>(0);
    const sensors::ImageDepth32F& depth_image_state =
        context.get_abstract_state<sensors::ImageDepth32F>(1);
    // label_image
    const auto& camera_base_pose_state =
        *dynamic_cast<const rendering::PoseVector<double>*>(
            context.get_discrete_state(0));

    camera_base_pose.get_mutable_value() = camera_base_pose_state.get_value();
    color_image = color_image_state;
    depth_image = depth_image_state;
    // label_image
  } else {
    impl_->DoCalcOutput(context.get_time(), x,
                        &camera_base_pose,
                        &color_image, &depth_image, nullptr /* &label_image*/);
  }
}

void RgbdCamera::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  DRAKE_DEMAND(impl_->is_discrete());
  const Eigen::VectorXd& x =
      this->EvalVectorInput(context, kPortStateInput)->CopyToVector();

  // Is there an easier way to do this???
  auto& camera_base_pose =
      *dynamic_cast<rendering::PoseVector<double>*>(
          state->get_mutable_discrete_state()->get_mutable_vector(0));

  // Outputs the image data.
  sensors::ImageBgra8U& color_image =
      state->get_mutable_abstract_state<sensors::ImageBgra8U>(0);

  sensors::ImageDepth32F& depth_image =
      state->get_mutable_abstract_state<sensors::ImageDepth32F>(1);
  // label_image

  impl_->DoCalcOutput(context.get_time(), x,
                      &camera_base_pose,
                      &color_image, &depth_image, nullptr /* &label_image */);
}

std::unique_ptr<DiscreteValues<double>> RgbdCamera::AllocateDiscreteState() const
{
  if (impl_->is_discrete()) {
   return std::make_unique<DiscreteValues<double>>(
       std::make_unique<rendering::PoseVector<double>>());
  } else {
    return LeafSystem<double>::AllocateDiscreteState();
  }
}

constexpr float RgbdCamera::InvalidDepth::kTooFar;
constexpr float RgbdCamera::InvalidDepth::kTooClose;

constexpr int16_t RgbdCamera::Label::kNoBody;
constexpr int16_t RgbdCamera::Label::kFlatTerrain;

RgbdCameraDirect::RgbdCameraDirect(const RigidBodyTree<double>& tree,
                                   const RigidBodyFrame<double>& frame,
                                   double fov_y,
                                   bool show_window)
    : impl_(new RgbdCamera::Impl(tree, frame,
                                 fov_y, show_window, true)) {
  impl_->set_depth_rel_noise_magnitude(0.02); // same as above
}

RgbdCameraDirect::~RgbdCameraDirect() {}

void RgbdCameraDirect::CalcImages(
    double t, const Eigen::VectorXd& x,
    rendering::PoseVector<double>* pcamera_base_pose, ImageBgra8U* pcolor_image,
    ImageDepth32F* pdepth_image, ImageLabel16I* plabel_image) {
  impl_->DoCalcOutput(t, x, pcamera_base_pose, pcolor_image,
                      pdepth_image, plabel_image);
}

const CameraInfo& RgbdCameraDirect::depth_camera_info() const {
  return impl_->depth_camera_info();
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake

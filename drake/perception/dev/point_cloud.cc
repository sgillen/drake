#include "point_cloud.h"

#include <iterator>

#include <fmt/format.h>

#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkFloatArray.h>

namespace drake {
namespace perception {

namespace {

template <typename T, typename... Args>
vtkSmartPointer<T> MakeVtk(Args&&... args) {
  return vtkSmartPointer<T>::New(std::forward<Args>(args)...);
}

const std::string kNameColors = "rgb_colors",
    kNameNormals = "normals",
    kNameCurvatures = "curvatures",
    kNameFeaturesPrefix = "features_";

std::string join(const std::vector<std::string>& elements,
                 const std::string &d) {
  std::ostringstream os;
  std::copy(elements.begin(), elements.end(),
            std::ostream_iterator<std::string>(os, d.c_str()));
  return os.str();
}

std::vector<std::string> ToStringVector(
    PointCloud::Capabilities c, const FeatureType& feature_type) {
  std::vector<std::string> out;
#define PC_CAPABILITY(flag, suffix) if (c & PointCloud::flag) \
    out.push_back(std::string(#flag) + suffix);
  PC_CAPABILITY(kXYZ, "");
  PC_CAPABILITY(kColors, "");
  PC_CAPABILITY(kNormals, "");
  PC_CAPABILITY(kCurvatures, "");
  PC_CAPABILITY(kFeatures, feature_type.name());
#undef PC_CAPABILITY
  return out;
}

std::string ToString(PointCloud::Capabilities c, const FeatureType &f) {
  return "(" + join(ToStringVector(c, f), " | ") + ")";
}

};

class PointCloud::Storage {
 public:
  Storage(const PointCloud* cloud)
      : cloud_(cloud) {
    // Allocate VTK point cloud.
    poly_data_ = vtkSmartPointer<vtkPolyData>::New();

    // See vtkPCLConversions.cxx, Cxx/PolyData/CopyAllArrays
    if (cloud_->has_xyz()) {
      // Why do they use `vtkNew`?
      auto points = vtkSmartPointer<vtkPoints>::New();
      points->SetDataTypeToFloat();
      points->SetNumberOfPoints(cloud_->size());
      poly_data_->SetPoints(points);
    }
    if (cloud_->has_colors()) {
      auto rgb_array = vtkSmartPointer<vtkUnsignedCharArray>::New();
      rgb_array->SetName(kNameColors.c_str());
      rgb_array->SetNumberOfTuples(cloud_->size());
      poly_data_->GetPointData()->AddArray(rgb_array);
    }
    if (cloud_->has_normals()) {
      auto normal_array = vtkSmartPointer<vtkFloatArray>::New();
      normal_array->SetName(kNameNormals.c_str());
      normal_array->SetNumberOfComponents(3);
      normal_array->SetNumberOfTuples(cloud_->size());
      poly_data_->GetPointData()->AddArray(normal_array);
    }
    if (cloud_->has_curvatures()) {
      auto curvature_array = vtkSmartPointer<vtkFloatArray>::New();
      curvature_array->SetName(kNameCurvatures.c_str());
      curvature_array->SetNumberOfComponents(1);
      curvature_array->SetNumberOfTuples(cloud_->size());
      poly_data_->GetPointData()->AddArray(curvature_array);
    }
    if (cloud_->has_features()) {
      auto feature_array = vtkSmartPointer<vtkFloatArray>::New();
      const std::string name =
          kNameFeaturesPrefix + cloud_->feature_type().name();
      feature_array->SetName(name.c_str());
      feature_array->SetNumberOfComponents(cloud_->feature_type().size());
      feature_array->SetNumberOfTuples(cloud_->size());
      poly_data_->GetPointData()->AddArray(feature_array);
    }
  }

  // Resize to parent cloud's size.
  void resize() {
    int old_size = poly_data_->GetNumberOfPoints();
    int new_size = cloud_->size();
  }

  float* get_xyz_data() {
    auto* points = poly_data_->GetPoints();
    DRAKE_ASSERT(points);
    return static_cast<float*>(points->GetVoidPointer(0));
  }

  uint8_t* get_color_data() {
    auto* colors = poly_data_->GetPointData()->GetArray(kNameColors.c_str());
    DRAKE_ASSERT(colors);
    return static_cast<uint8_t*>(colors->GetVoidPointer(0));
  }

 private:
  const PointCloud* cloud_{};
  vtkSmartPointer<vtkPolyData> poly_data_;
};

namespace {

PointCloud::Capabilities ResolveCapabilities(const PointCloud& other,
                         PointCloud::Capabilities in) {
  if (in == PointCloud::kInherit) {
    return other.capabilities();
  } else {
    return in;
  }
}

// If kFeatureInherit is used, then we take on the other's type is used.
// If another feature is used, and we wish to copy the other's features,
// ensure they are compatible.
FeatureType ResolveFeatureType(const PointCloud& other,
                               PointCloud::Capabilities in,
                               const FeatureType& feature_type) {
  if (feature_type == kFeatureInherit) {
    return other.feature_type();
  } else {
    if (in & PointCloud::kFeatures) {
      DRAKE_DEMAND(feature_type == other.feature_type());
    }
    return feature_type;
  }
}

}  // namespace

PointCloud::PointCloud(
    PointCloud::Index new_size,
    PointCloud::Capabilities capabilities,
    const FeatureType& feature_type)
    : size_(new_size),
      capabilities_(capabilities),
      feature_type_(feature_type) {
  DRAKE_DEMAND(!(capabilities & PointCloud::kInherit));
  DRAKE_DEMAND(feature_type_ != kFeatureInherit);
  storage_.reset(new Storage(this));
}

PointCloud::PointCloud(const PointCloud& other,
                       PointCloud::Capabilities copy_capabilities,
                       const FeatureType& feature_type)
    : PointCloud(other.size(),
                 ResolveCapabilities(other, copy_capabilities),
                 ResolveFeatureType(other, copy_capabilities, feature_type)) {}

void PointCloud::resize(PointCloud::Index new_size) {
  DRAKE_DEMAND(new_size >= 0);
  int old_size = size();
  size_ = new_size;
  storage_->resize();
  if (new_size > old_size) {
    int size_diff = new_size - old_size;
    SetDefault(old_size, size_diff);
  }
}

void PointCloud::SetDefault(int start, int num) {
  auto set = [=](auto ref, auto value) {
    ref.middleCols(start, num).setConstant(value);
  };
  if (has_xyz()) {
    set(mutable_xyz(), kDefaultValue);
  }
  if (has_colors()) {
    set(mutable_colors(), kDefaultColor);
  }
  if (has_normals()) {
    set(mutable_normals(), kDefaultValue);
  }
  if (has_curvatures()) {
    set(mutable_curvatures(), kDefaultValue);
  }
  if (has_features()) {
    set(mutable_features(), kDefaultValue);
  }
}

void PointCloud::MergeFrom(const PointCloud& other) {
  int old_size = size();
  int new_size = old_size + other.size();
  resize(new_size);
  CopyFrom(other);
}
void PointCloud::CopyFrom(const PointCloud& other,
                          PointCloud::Capabilities c,
                          bool allow_subset,
                          bool allow_resize) {

}

void PointCloud::AddPoints(
    int add_size,
    bool skip_initialization) {
  int old_size = size();
  int new_size = old_size + add_size;
  resize(new_size);
  if (!skip_initialization) {
    SetDefault(old_size, add_size);
  }
}

bool PointCloud::has_xyz() const {
  return capabilities_ | kXYZ;
}
bool PointCloud::has_colors() const {
  return capabilities_ | kColors;
}
bool PointCloud::has_normals() const {
  return capabilities_ | kNormals;
}
bool PointCloud::has_curvatures() const {
  return capabilities_ | kCurvatures;
}
bool PointCloud::has_features() const {
  return capabilities_ | kFeatures;
}
bool PointCloud::has_features(const FeatureType& feature_type) const {
  return has_features() && feature_type_ == feature_type;
}

Eigen::Ref<const Matrix3X<T>> PointCloud::xyz() const {
  return Eigen::Map<const Matrix3X<T>>(storage_->get_xyz_data(),
                                       3, size());
}

void PointCloud::RequireCapabilities(
    PointCloud::Capabilities c,
    const FeatureType& f) {
  bool good = true;
  if (capabilities() & c != c) {
    good = false;
  } else if (c | PointCloud::kFeatures) {
    DRAKE_DEMAND(f != kFeatureNone);
    DRAKE_DEMAND(f != kFeatureInherit);
    if (feature_type() != f) {
      good = false;
    }
  }
  if (!good) {
    throw std::runtime_error(
        fmt::format("PointCloud does not have expected capabilities.\n"
                    "expected {}, got {}",
                    ToString(c, f),
                    ToString(capabilities(), feature_type())));
  }
}

}  // namespace perception

}  // namespace drake


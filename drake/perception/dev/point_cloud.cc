#include "point_cloud.h"

#include <iterator>

#include <fmt/format.h>

#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkFloatArray.h>

using Eigen::Map;

namespace drake {
namespace perception {

typedef PointCloud::T T;
typedef PointCloud::F F;
constexpr int NC = PointCloud::NC;
typedef PointCloud::C C;

namespace {

template <typename T, typename... Args>
vtkSmartPointer<T> MakeVtk(Args&&... args) {
  return vtkSmartPointer<T>::New(std::forward<Args>(args)...);
}

const std::string kNameColors = "rgb8u_colors",
    kNameNormals = "normals",
    kNameFeaturesPrefix = "features_";

std::string join(const std::vector<std::string>& elements,
                 const std::string &d) {
  std::ostringstream os;
  std::copy(elements.begin(), elements.end(),
            std::ostream_iterator<std::string>(os, d.c_str()));
  return os.str();
}

std::vector<std::string> ToStringVector(
    PointCloud::CapabilitySet c, const FeatureType& feature_type) {
  std::vector<std::string> out;
#define PC_CAPABILITY(flag, suffix) \
    if (c & PointCloud::flag) \
      out.push_back(std::string(#flag) + suffix);
  PC_CAPABILITY(kXYZ, "");
  PC_CAPABILITY(kColor, "");
  PC_CAPABILITY(kNormal, "");
  PC_CAPABILITY(kFeature, "_" + feature_type.name());
#undef PC_CAPABILITY
  return out;
}

std::string ToString(PointCloud::CapabilitySet c, const FeatureType &f) {
  return "(" + join(ToStringVector(c, f), " | ") + ")";
}

template <typename XprType>
Eigen::Map<XprType> VtkToEigen(vtkDataArray *array) {
  // Follow suite with vtk_to_numpy.
  typedef typename XprType::Scalar T;
  T* raw = static_cast<T*>(array->GetVoidPointer(0));
  const int nrows = array->GetNumberOfComponents();
  const int ncols = array->GetNumberOfTuples();
  return Eigen::Map<XprType>(raw, nrows, ncols);
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
    if (cloud_->has_color()) {
      auto rgb_array = vtkSmartPointer<vtkUnsignedCharArray>::New();
      rgb_array->SetName(kNameColors.c_str());
      rgb_array->SetNumberOfComponents(NC);
      rgb_array->SetNumberOfTuples(cloud_->size());
      poly_data_->GetPointData()->AddArray(rgb_array);
    }
    if (cloud_->has_normal()) {
      auto normal_array = vtkSmartPointer<vtkFloatArray>::New();
      normal_array->SetName(kNameNormals.c_str());
      normal_array->SetNumberOfComponents(3);
      normal_array->SetNumberOfTuples(cloud_->size());
      poly_data_->GetPointData()->AddArray(normal_array);
    }
    if (cloud_->has_feature()) {
      auto feature_array = vtkSmartPointer<vtkFloatArray>::New();
      feature_array->SetName(feature_name().c_str());
      feature_array->SetNumberOfComponents(cloud_->feature_type().size());
      feature_array->SetNumberOfTuples(cloud_->size());
      poly_data_->GetPointData()->AddArray(feature_array);
    }
  }

  std::string feature_name() const {
    return kNameFeaturesPrefix + cloud_->feature_type().name();
  }

  int size() const {
    if (cloud_->has_xyz()) {
      return poly_data_->GetNumberOfPoints();
    } else {
      // TODO(eric.cousineau): See if there is a more elegant way of
      // supporting non-XYZ point clouds (like PCL's PointCloud<Normal>).
      return poly_data_->GetPointData()->GetArray(0)->GetNumberOfTuples();
    }
  }

  // Resize to parent cloud's size.
  void resize() {
//    int old_size = poly_data_->GetNumberOfPoints();
    int new_size = cloud_->size();

    if (cloud_->has_xyz()) {
      GetPoints()->Resize(new_size);
      // Dunno why, but we have to propagate this size???
      GetPoints()->SetNumberOfPoints(new_size);
    }
    if (cloud_->has_color()) {
      GetColors()->Resize(new_size);
      GetColors()->SetNumberOfTuples(new_size);
    }
    if (cloud_->has_normal()) {
      GetNormals()->Resize(new_size);
      GetNormals()->SetNumberOfTuples(new_size);
    }
    if (cloud_->has_feature()) {
      GetFeatures()->Resize(new_size);
      GetFeatures()->SetNumberOfTuples(new_size);
    }

    poly_data_->GetPointData()->SetNumberOfTuples(new_size);

    CheckInvariants();
  }

  Eigen::Map<Matrix3X<T>> xyzs() {
    DRAKE_ASSERT(cloud_->has_xyz());
    float* raw = static_cast<float*>(GetPoints()->GetVoidPointer(0));
    Eigen::Map<Matrix3X<T>> out(raw, 3, size());
    return out;
  }

  Eigen::Map<MatrixNX<NC, C>> colors() {
    DRAKE_ASSERT(cloud_->has_color());
    return VtkToEigen<MatrixNX<NC, C>>(GetColors());
  }

  Eigen::Map<Matrix3X<T>> normals() {
    DRAKE_ASSERT(cloud_->has_normal());
    return VtkToEigen<Matrix3X<T>>(GetNormals());
  }

  Eigen::Map<MatrixX<T>> features() {
    DRAKE_ASSERT(cloud_->has_feature());
    return VtkToEigen<MatrixX<T>>(GetFeatures());
  }

 private:
  vtkPoints* GetPoints() const {
    return poly_data_->GetPoints();
  }
  vtkDataArray* GetColors() const {
    return poly_data_->GetPointData()->GetArray(kNameColors.c_str());
  }
  vtkDataArray* GetNormals() const {
    return poly_data_->GetPointData()->GetArray(kNameNormals.c_str());
  }
  vtkDataArray* GetFeatures() const {
    return poly_data_->GetPointData()->GetArray(feature_name().c_str());
  }

  void CheckInvariants() const {
    int cloud_size = cloud_->size();
    DRAKE_DEMAND(size() == cloud_->size());
    if (cloud_->has_xyz()) {
      int xyz_size = GetPoints()->GetNumberOfPoints();
      DRAKE_DEMAND(xyz_size == cloud_size);
    }
    if (cloud_->has_color()) {
      int color_size = GetColors()->GetNumberOfTuples();
      DRAKE_DEMAND(color_size == cloud_size);
    }
    if (cloud_->has_normal()) {
      int normal_size = GetNormals()->GetNumberOfTuples();
      DRAKE_DEMAND(normal_size == cloud_size);
    }
  }

  const PointCloud* cloud_{};
  vtkSmartPointer<vtkPolyData> poly_data_;
};

namespace {

// Ensure that a capability set is complete valid (does not have extra bits).
void ValidateCapabilities(PointCloud::CapabilitySet c) {
  PointCloud::CapabilitySet full_mask =
      PointCloud::kXYZ | PointCloud::kNormal | PointCloud::kColor |
          PointCloud::kFeature;
  if (c <= 0 || c > full_mask) {
    throw std::runtime_error("Invalid CapabilitySet");
  }
}

PointCloud::CapabilitySet ResolveCapabilities(const PointCloud& other,
                         PointCloud::CapabilitySet in) {
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
                               PointCloud::CapabilitySet in,
                               const FeatureType& feature_type) {
  if (feature_type == kFeatureInherit) {
    return other.feature_type();
  } else {
    if (in & PointCloud::kFeature) {
      DRAKE_DEMAND(feature_type == other.feature_type());
    }
    return feature_type;
  }
}

}  // namespace

PointCloud::PointCloud(
    PointCloud::Index new_size,
    PointCloud::CapabilitySet capabilities,
    const FeatureType& feature_type)
    : size_(new_size),
      capabilities_(capabilities),
      feature_type_(feature_type) {
  ValidateCapabilities(capabilities_);
  DRAKE_DEMAND(!(capabilities_ & PointCloud::kInherit));
  DRAKE_DEMAND(feature_type_ != kFeatureInherit);
  if (has_feature()) {
    DRAKE_DEMAND(feature_type_ != kFeatureNone);
  }
  storage_.reset(new Storage(this));
  SetDefault(0, size_);
}

PointCloud::PointCloud(const PointCloud& other,
                       PointCloud::CapabilitySet copy_capabilities,
                       const FeatureType& feature_type)
    : PointCloud(other.size(),
                 ResolveCapabilities(other, copy_capabilities),
                 ResolveFeatureType(other, copy_capabilities, feature_type)) {}

// Define destructor here to use complete definition of `Storage`.
PointCloud::~PointCloud() {}

void PointCloud::Resize(PointCloud::Index new_size) {
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
    set(mutable_xyzs(), kDefaultValue);
  }
  if (has_color()) {
    set(mutable_colors(), kDefaultColor);
  }
  if (has_normal()) {
    set(mutable_normals(), kDefaultValue);
  }
  if (has_feature()) {
    set(mutable_features(), kDefaultValue);
  }
}

void PointCloud::MergeFrom(const PointCloud& other) {
  int old_size = size();
  int new_size = old_size + other.size();
  Resize(new_size);
  CopyFrom(other);
}

void PointCloud::CopyFrom(const PointCloud& other,
                          PointCloud::CapabilitySet c,
                          bool allow_subset,
                          bool allow_resize) {
  // TODO(eric.cousineau): Actually USE `c`!
  int old_size = size();
  int new_size = other.size();
  if (!allow_subset) {
    // Ensure that this point cloud has all of the same capabilities as the
    // other.
    RequireCapabilities(other.capabilities(), other.feature_type());
  } else {
  }
  if (allow_resize) {
    Resize(new_size);
  } else if (new_size != old_size) {
    throw std::runtime_error(
        fmt::format("CopyFrom: {} != {}", new_size, old_size));
  }
  if (has_xyz() && other.has_xyz()) {
    mutable_xyzs() = other.xyzs();
  }
  if (has_color() && other.has_color()) {
    mutable_colors() = other.colors();
  }
  if (has_normal() && other.has_normal()) {
    mutable_normals() = other.normals();
  }
  if (has_feature() && other.has_feature()) {
    // Ensure that we have the correct feature.
    DRAKE_DEMAND(feature_type() == other.feature_type());
    mutable_features() = other.features();
  }
}

void PointCloud::AddPoints(
    int add_size,
    bool skip_initialization) {
  int old_size = size();
  int new_size = old_size + add_size;
  Resize(new_size);
  if (!skip_initialization) {
    SetDefault(old_size, add_size);
  }
}

bool PointCloud::has_xyz() const {
  return capabilities_ & kXYZ;
}
Eigen::Ref<const Matrix3X<T>> PointCloud::xyzs() const {
  return storage_->xyzs();
}
Eigen::Ref<Matrix3X<T>> PointCloud::mutable_xyzs() {
  return storage_->xyzs();
}

bool PointCloud::has_color() const {
  return capabilities_ & kColor;
}
Eigen::Ref<const MatrixNX<NC, C>> PointCloud::colors() const {
  return storage_->colors();
}
Eigen::Ref<MatrixNX<NC, C>> PointCloud::mutable_colors() {
  return storage_->colors();
}

bool PointCloud::has_normal() const {
  return capabilities_ & kNormal;
}
Eigen::Ref<const Matrix3X<T>> PointCloud::normals() const {
  return storage_->normals();
}
Eigen::Ref<Matrix3X<T>> PointCloud::mutable_normals() {
  return storage_->normals();
}

bool PointCloud::has_feature() const {
  return capabilities_ & kFeature;
}
bool PointCloud::has_feature(const FeatureType& feature_type) const {
  return has_feature() && feature_type_ == feature_type;
}
Eigen::Ref<const MatrixX<F>> PointCloud::features() const {
  return storage_->features();
}
Eigen::Ref<MatrixX<F>> PointCloud::mutable_features() {
  return storage_->features();
}

bool PointCloud::HasCapabilities(
    PointCloud::CapabilitySet c,
    const FeatureType& f) const {
  bool good = true;
  if ((capabilities() & c) != c) {
    good = false;
  } else if (c & PointCloud::kFeature) {
    DRAKE_DEMAND(f != kFeatureNone);
    DRAKE_DEMAND(f != kFeatureInherit);
    if (feature_type() != f) {
      good = false;
    }
  }
  return good;
}

void PointCloud::RequireCapabilities(
    CapabilitySet c,
    const FeatureType& f) const {
  if (!HasCapabilities(c, f)) {
    throw std::runtime_error(
        fmt::format("PointCloud does not have expected capabilities.\n"
                    "Expected {}, got {}",
                    ToString(c, f),
                    ToString(capabilities(), feature_type())));
  }
}

bool PointCloud::HasExactCapabilities(
      CapabilitySet c,
      const FeatureType& f) const {
  return HasCapabilities(c, f) && capabilities() == c;
}

void PointCloud::RequireExactCapabilities(
    CapabilitySet c,
    const FeatureType& f) const {
  if (!HasExactCapabilities(c, f)) {
    throw std::runtime_error(
        fmt::format("PointCloud does not have the exact expected capabilities."
                    "\nExpected {}, got {}",
                    ToString(c, f),
                    ToString(capabilities(), feature_type())));
  }
}

}  // namespace perception

}  // namespace drake


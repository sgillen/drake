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

namespace {

// Convenience aliases.
typedef PointCloud::T T;
typedef PointCloud::F F;
constexpr int NC = PointCloud::NC;
typedef PointCloud::C C;

// Names.
const std::string kNameColors = "rgb8u_colors",
    kNameNormals = "normals",
    kNameFeatures = "features";

// Utility.
std::string join(const std::vector<std::string>& elements,
                 const std::string &d) {
  std::ostringstream os;
  for (size_t i = 0; i < elements.size(); ++i) {
    os << elements[i];
    if (i + 1 < elements.size())
      os << d;
  }
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
  PC_CAPABILITY(kFeature, "::" + feature_type.name());
#undef PC_CAPABILITY
  return out;
}

// Simple VTK utility, following suite with `vtk_to_numpy`.
template <typename XprType>
Eigen::Map<XprType> VtkToEigen(vtkDataArray *array) {
  typedef typename XprType::Scalar T;
  T* raw = static_cast<T*>(array->GetVoidPointer(0));
  const int nrows = array->GetNumberOfComponents();
  const int ncols = array->GetNumberOfTuples();
  return Eigen::Map<XprType>(raw, nrows, ncols);
}

// Simple VTK utility, following suite with `vtk_to_numpy`.
template <typename T>
Eigen::Map<Matrix3X<T>> VtkToEigen(vtkPoints *array) {
  T* raw = static_cast<T*>(array->GetVoidPointer(0));
  const int ncols = array->GetNumberOfPoints();
  return Eigen::Map<Matrix3X<T>>(raw, 3, ncols);
}

};

class PointCloud::Storage {
 public:
  Storage(int new_size, CapabilitySet& c, const FeatureType& f)
      : capabilities_(c) {
    // Allocate VTK point cloud.
    poly_data_ = vtkSmartPointer<vtkPolyData>::New();
    // TODO(eric.cousineau): Cache these results if it improves performance.
    // (Access to VTK array, store Eigen::Map in an EigenPtr, etc.)
    // @ref vtkPCLConversions.cxx, Cxx/PolyData/CopyAllArrays
    if (capabilities_ & kXYZ) {
      auto points = vtkSmartPointer<vtkPoints>::New();
      points->SetDataTypeToFloat();
      points->SetNumberOfPoints(new_size);
      poly_data_->SetPoints(points);
    }
    if (capabilities_ & kColor) {
      auto color_array = vtkSmartPointer<vtkUnsignedCharArray>::New();
      color_array->SetName(kNameColors.c_str());
      color_array->SetNumberOfComponents(NC);
      color_array->SetNumberOfTuples(new_size);
      poly_data_->GetPointData()->AddArray(color_array);
    }
    if (capabilities_ & kNormal) {
      auto normal_array = vtkSmartPointer<vtkFloatArray>::New();
      normal_array->SetName(kNameNormals.c_str());
      normal_array->SetNumberOfComponents(3);
      normal_array->SetNumberOfTuples(new_size);
      poly_data_->GetPointData()->AddArray(normal_array);
    }
    if (capabilities_ & kFeature) {
      auto feature_array = vtkSmartPointer<vtkFloatArray>::New();
      feature_array->SetName(kNameFeatures.c_str());
      feature_array->SetNumberOfComponents(f.size());
      feature_array->SetNumberOfTuples(new_size);
      poly_data_->GetPointData()->AddArray(feature_array);
    }
  }

  int size() const {
    // TODO(eric.cousineau): See if there is a more elegant way of
    // supporting non-XYZ point clouds (like PCL's PointCloud<Normal>).
    if (capabilities_ & kXYZ) {
      return poly_data_->GetNumberOfPoints();
    } else {
      DRAKE_DEMAND(poly_data_->GetPointData()->GetNumberOfArrays() > 0);
      return poly_data_->GetPointData()->GetArray(0)->GetNumberOfTuples();
    }
  }

  // Resize to parent cloud's size.
  void resize(int new_size) {
    if (capabilities_ & kXYZ) {
      GetPoints()->Resize(new_size);
      // Dunno why, but we have to propagate this size???
      GetPoints()->SetNumberOfPoints(new_size);
    }
    if (capabilities_ & kColor) {
      GetColors()->Resize(new_size);
      GetColors()->SetNumberOfTuples(new_size);
    }
    if (capabilities_ & kNormal) {
      GetNormals()->Resize(new_size);
      GetNormals()->SetNumberOfTuples(new_size);
    }
    if (capabilities_ & kFeature) {
      GetFeatures()->Resize(new_size);
      GetFeatures()->SetNumberOfTuples(new_size);
    }

    poly_data_->GetPointData()->SetNumberOfTuples(new_size);

    CheckInvariants();
  }

  Eigen::Map<Matrix3X<T>> xyzs() {
    return VtkToEigen<T>(GetPoints());
  }

  Eigen::Map<MatrixNX<NC, C>> colors() {
    return VtkToEigen<MatrixNX<NC, C>>(GetColors());
  }

  Eigen::Map<Matrix3X<T>> normals() {
    return VtkToEigen<Matrix3X<T>>(GetNormals());
  }

  Eigen::Map<MatrixX<T>> features() {
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
    return poly_data_->GetPointData()->GetArray(kNameFeatures.c_str());
  }

  void CheckInvariants() const {
    if (capabilities_ & kXYZ) {
      const int xyz_size = GetPoints()->GetNumberOfPoints();
      DRAKE_DEMAND(xyz_size == size());
    }
    if (capabilities_ & kColor) {
      const int color_size = GetColors()->GetNumberOfTuples();
      DRAKE_DEMAND(color_size == size());
    }
    if (capabilities_ & kNormal) {
      const int normal_size = GetNormals()->GetNumberOfTuples();
      DRAKE_DEMAND(normal_size == size());
    }
    if (capabilities_ & kFeature) {
      const int feature_size = GetFeatures()->GetNumberOfTuples();
      DRAKE_DEMAND(feature_size == size());
    }
  }

  const CapabilitySet capabilities_;
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

std::string ToString(PointCloud::CapabilitySet c, const FeatureType& f) {
  return "(" + join(ToStringVector(c, f), " | ") + ")";
}

PointCloud::PointCloud(
    PointCloud::Index new_size,
    PointCloud::CapabilitySet capabilities,
    const FeatureType& feature_type)
    : size_(new_size),
      capabilities_(capabilities),
      feature_type_(feature_type) {
  ValidateCapabilities(capabilities_);
  if (capabilities_ & PointCloud::kInherit)
    throw std::runtime_error("Cannot construct a PointCloud with kInherit");
  if (feature_type == kFeatureInherit)
    throw std::runtime_error(
        "Cannot construct a PointCloud with kFeatureInherit");
  if (has_features()) {
    if (feature_type_ == kFeatureNone)
      throw std::runtime_error("Cannot specify kFeatureNone with kFeature");
  }
  else if (feature_type != kFeatureNone)
    throw std::runtime_error(
        "Must specify kFeatureNone if kFeature is not present");

  storage_.reset(new Storage(new_size, capabilities, feature_type));
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

void PointCloud::resize(PointCloud::Index new_size) {
  DRAKE_DEMAND(new_size >= 0);
  int old_size = size();
  size_ = new_size;
  storage_->resize(new_size);
  DRAKE_DEMAND(storage_->size() == new_size);
  if (new_size > old_size) {
    int size_diff = new_size - old_size;
    SetDefault(old_size, size_diff);
  }
}

void PointCloud::SetDefault(int start, int num) {
  auto set = [=](auto ref, auto value) {
    ref.middleCols(start, num).setConstant(value);
  };
  if (has_xyzs()) {
    set(mutable_xyzs(), kDefaultValue);
  }
  if (has_colors()) {
    set(mutable_colors(), kDefaultColor);
  }
  if (has_normals()) {
    set(mutable_normals(), kDefaultValue);
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
    // TODO(eric.cousineau): Fill this in.
  }
  if (allow_resize) {
    resize(new_size);
  } else if (new_size != old_size) {
    throw std::runtime_error(
        fmt::format("CopyFrom: {} != {}", new_size, old_size));
  }
  if (has_xyzs() && other.has_xyzs()) {
    mutable_xyzs() = other.xyzs();
  }
  if (has_colors() && other.has_colors()) {
    mutable_colors() = other.colors();
  }
  if (has_normals() && other.has_normals()) {
    mutable_normals() = other.normals();
  }
  if (has_features() && other.has_features()) {
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
  resize(new_size);
  if (!skip_initialization) {
    SetDefault(old_size, add_size);
  }
}

bool PointCloud::has_xyzs() const {
  return capabilities_ & kXYZ;
}
Eigen::Ref<const Matrix3X<T>> PointCloud::xyzs() const {
  DRAKE_DEMAND(has_xyzs());
  return storage_->xyzs();
}
Eigen::Ref<Matrix3X<T>> PointCloud::mutable_xyzs() {
  DRAKE_DEMAND(has_xyzs());
  return storage_->xyzs();
}

bool PointCloud::has_colors() const {
  return capabilities_ & kColor;
}
Eigen::Ref<const MatrixNX<NC, C>> PointCloud::colors() const {
  DRAKE_DEMAND(has_colors());
  return storage_->colors();
}
Eigen::Ref<MatrixNX<NC, C>> PointCloud::mutable_colors() {
  DRAKE_DEMAND(has_colors());
  return storage_->colors();
}

bool PointCloud::has_normals() const {
  return capabilities_ & kNormal;
}
Eigen::Ref<const Matrix3X<T>> PointCloud::normals() const {
  DRAKE_DEMAND(has_normals());
  return storage_->normals();
}
Eigen::Ref<Matrix3X<T>> PointCloud::mutable_normals() {
  DRAKE_DEMAND(has_normals());
  return storage_->normals();
}

bool PointCloud::has_features() const {
  return capabilities_ & kFeature;
}
bool PointCloud::has_features(const FeatureType& feature_type) const {
  return has_features() && feature_type_ == feature_type;
}
Eigen::Ref<const MatrixX<F>> PointCloud::features() const {
  DRAKE_DEMAND(has_features());
  return storage_->features();
}
Eigen::Ref<MatrixX<F>> PointCloud::mutable_features() {
  DRAKE_DEMAND(has_features());
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


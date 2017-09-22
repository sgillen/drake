#include "point_cloud.h"

#include <iterator>

#include <fmt/format.h>

#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkFloatArray.h>

using Eigen::Map;
using Eigen::NoChange;

namespace drake {
namespace perception {

namespace {

// Convenience aliases.
typedef PointCloud::T T;
typedef PointCloud::F F;
constexpr int NC = PointCloud::NC;
typedef PointCloud::C C;

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

};

// Provides encapsulated storage
class PointCloud::Storage {
 public:
  Storage(int new_size, CapabilitySet& c, const FeatureType&)
      : capabilities_(c) {
    resize(new_size);
  }

  int size() const { return size_; }

  // Resize to parent cloud's size.
  void resize(int new_size) {
    size_ = new_size;
    if (capabilities_ & kXYZ) {
      xyzs_.conservativeResize(NoChange, new_size);
    }
    if (capabilities_ & kColor) {
      colors_.conservativeResize(NoChange, new_size);
    }
    if (capabilities_ & kNormal) {
      normals_.conservativeResize(NoChange, new_size);
    }
    if (capabilities_ & kFeature) {
      features_.conservativeResize(NoChange, new_size);
    }

    CheckInvariants();
  }

  Eigen::Map<Matrix3X<T>> xyzs() {
    return xyzs_;
  }

  Eigen::Map<MatrixNX<NC, C>> colors() {
    return colors_;
  }

  Eigen::Map<Matrix3X<T>> normals() {
    return normals_;
  }

  Eigen::Map<MatrixX<T>> features() {
    return features_;
  }

  // Synchronize contents if this storage structure was changed independent of
  // the owning `PointCloud`.
  void Sync() {
    // Ensure that
    CheckInvariants();
  }

 private:
  void CheckInvariants() const {
    if (capabilities_ & kXYZ) {
      const int xyz_size = xyzs_.cols();
      DRAKE_DEMAND(xyz_size == size());
    }
    if (capabilities_ & kColor) {
      const int color_size = colors_.cols();
      DRAKE_DEMAND(color_size == size());
    }
    if (capabilities_ & kNormal) {
      const int normal_size = normals_.cols();
      DRAKE_DEMAND(normal_size == size());
    }
    if (capabilities_ & kFeature) {
      const int feature_size = features_.cols();
      DRAKE_DEMAND(feature_size == size());
    }
  }

  const CapabilitySet capabilities_;
  int size_{};
  Matrix3X<T> xyzs_;
  MatrixNX<NC, C> colors_;
  Matrix3X<T> normals_;
  MatrixX<T> features_;
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


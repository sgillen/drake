#include "point_cloud.h"

#include <sstream>

#include <fmt/format.h>

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

// Utility for `ToString`.
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

// Convert a PointCloud's set of capabilities to a string vector (for use
// with `ToString`).
std::vector<std::string> ToStringVector(
    PointCloud::CapabilitySet c, const FeatureType& feature_type) {
  std::vector<std::string> out;
  if (c & PointCloud::kXYZ)
    out.push_back("kXYZ");
  if (c & PointCloud::kColor)
      out.push_back("kColor");
  if (c & PointCloud::kNormal)
      out.push_back("kNormal");
  if (c & PointCloud::kFeature)
      out.push_back("kFeature::" + feature_type.name());
  return out;
}

};

// Provides encapsulated storage for a `PointCloud`.
class PointCloud::Storage {
 public:
  Storage(int new_size, CapabilitySet& c, const FeatureType&)
      : capabilities_(c) {
    resize(new_size);
  }

  // Returns size of the storage.
  int size() const { return size_; }

  // Resize to parent cloud's size.
  void resize(int new_size) {
    size_ = new_size;
    if (capabilities_ & kXYZ)
      xyzs_.conservativeResize(NoChange, new_size);
    if (capabilities_ & kColor)
      colors_.conservativeResize(NoChange, new_size);
    if (capabilities_ & kNormal)
      normals_.conservativeResize(NoChange, new_size);
    if (capabilities_ & kFeature)
      features_.conservativeResize(NoChange, new_size);
    CheckInvariants();
  }

  Eigen::Map<Matrix3X<T>> xyzs() { return xyzs_; }
  Eigen::Map<MatrixNX<NC, C>> colors() { return colors_; }
  Eigen::Map<Matrix3X<T>> normals() { return normals_; }
  Eigen::Map<MatrixX<T>> features() { return features_; }

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

// Implements the rules set forth in `SetFrom`.
// @pre Valid point clouds `a` and `b`.
// @post The returned capabilities will be valid for both point clouds. If
//   `c` enables features, then the feature type will be shared by both point
//   clouds.
PointCloud::CapabilitySet ResolveCapabilities(
    const PointCloud& a,
    const PointCloud& b,
    PointCloud::CapabilitySet c) {
  if (c == PointCloud::kInherit) {
    // If we do not permit a subset, expect the exact same capabilities.
    a.RequireExactCapabilities(b.capabilities(), b.feature_type());
    return a.capabilities();
  } else {
    FeatureType f = kFeatureNone;
    if (c & PointCloud::kFeature)
      f = a.feature_type();
    a.RequireCapabilities(c, f);
    b.RequireCapabilities(c, f);
    return c;
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

void PointCloud::resize(PointCloud::Index new_size, bool skip_initialization) {
  DRAKE_DEMAND(new_size >= 0);
  int old_size = size();
  size_ = new_size;
  storage_->resize(new_size);
  DRAKE_DEMAND(storage_->size() == new_size);
  if (new_size > old_size && !skip_initialization) {
    int size_diff = new_size - old_size;
    SetDefault(old_size, size_diff);
  }
  if (new_size != old_size && has_image_dimensions()) {
    image_dimensions_ = nullopt;
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

void PointCloud::SetFrom(const PointCloud& other,
                         PointCloud::CapabilitySet c,
                         bool allow_resize) {
  int old_size = size();
  int new_size = other.size();
  if (allow_resize) {
    resize(new_size);
  } else if (new_size != old_size) {
    throw std::runtime_error(
        fmt::format("CopyFrom: {} != {}", new_size, old_size));
  }
  CapabilitySet c_final = ResolveCapabilities(*this, other, c);
  if (c_final & kXYZ) {
    mutable_xyzs() = other.xyzs();
  }
  if (c_final & kColor) {
    mutable_colors() = other.colors();
  }
  if (c_final & kNormal) {
    mutable_normals() = other.normals();
  }
  if (c_final & kFeature) {
    mutable_features() = other.features();
  }
}

void PointCloud::AddPoints(
    int add_size,
    bool skip_initialization) {
  DRAKE_DEMAND(add_size >= 0);
  const int new_size = size() + add_size;
  resize(new_size, skip_initialization);
}

void PointCloud::AddPoints(const PointCloud& other, CapabilitySet c) {
  int old_size = size();
  int new_size = old_size + other.size();
  resize(new_size);
  c = ResolveCapabilities(*this, other, c);
  // Get the block corresponding to the newly allocated values.
  auto fresh_block = [=](auto value) {
    return value.middleCols(old_size, other.size());
  };
  if (c & kXYZ) {
    fresh_block(mutable_xyzs()) = other.xyzs();
  }
  if (c & kNormal) {
    fresh_block(mutable_normals()) = other.normals();
  }
  if (c & kColor) {
    fresh_block(mutable_colors()) = other.colors();
  }
  if (c & kFeature) {
    fresh_block(mutable_features()) = other.features();
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
  } else {
    if (c & PointCloud::kFeature) {
      DRAKE_DEMAND(f != kFeatureNone);
      DRAKE_DEMAND(f != kFeatureInherit);
      if (feature_type() != f) {
        good = false;
      }
    } else {
      DRAKE_DEMAND(f == kFeatureNone);
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

bool PointCloud::has_image_dimensions() const {
  return static_cast<bool>(image_dimensions_);
}
void PointCloud::set_image_dimensions(const ImageDimensions& dim) {
  DRAKE_DEMAND(dim.size() == size());
  image_dimensions_ = dim;
}
ImageDimensions PointCloud::image_dimensions() const {
  DRAKE_DEMAND(has_image_dimensions());
  return *image_dimensions_;
}

}  // namespace perception
}  // namespace drake

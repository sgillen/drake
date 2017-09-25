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
    PointCloud::CapabilitySet c, const ExtraType& feature_type) {
  std::vector<std::string> out;
  if (c & PointCloud::kXYZs)
    out.push_back("kXYZs");
  if (c & PointCloud::kColors)
      out.push_back("kColors");
  if (c & PointCloud::kNormals)
      out.push_back("kNormals");
  if (c & PointCloud::kFeatures)
      out.push_back("kFeatures::" + feature_type.name());
  return out;
}

};

// Provides encapsulated storage for a `PointCloud`.
class PointCloud::Storage {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Storage)

  Storage(int new_size, CapabilitySet& c, const ExtraType&)
      : capabilities_(c) {
    resize(new_size);
  }

  // Returns size of the storage.
  int size() const { return size_; }

  // Resize to parent cloud's size.
  void resize(int new_size) {
    size_ = new_size;
    if (capabilities_ & kXYZs)
      xyzs_.conservativeResize(NoChange, new_size);
    if (capabilities_ & kColors)
      colors_.conservativeResize(NoChange, new_size);
    if (capabilities_ & kNormals)
      normals_.conservativeResize(NoChange, new_size);
    if (capabilities_ & kFeatures)
      features_.conservativeResize(NoChange, new_size);
    CheckInvariants();
  }

  Eigen::Ref<Matrix3X<T>> xyzs() { return xyzs_; }
  Eigen::Ref<MatrixNX<NC, C>> colors() { return colors_; }
  Eigen::Ref<Matrix3X<T>> normals() { return normals_; }
  Eigen::Ref<MatrixX<T>> features() { return features_; }

 private:
  void CheckInvariants() const {
    if (capabilities_ & kXYZs) {
      const int xyz_size = xyzs_.cols();
      DRAKE_DEMAND(xyz_size == size());
    }
    if (capabilities_ & kColors) {
      const int color_size = colors_.cols();
      DRAKE_DEMAND(color_size == size());
    }
    if (capabilities_ & kNormals) {
      const int normal_size = normals_.cols();
      DRAKE_DEMAND(normal_size == size());
    }
    if (capabilities_ & kFeatures) {
      const int feature_size = features_.cols();
      DRAKE_DEMAND(feature_size == size());
    }
  }

  CapabilitySet capabilities_;
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
      PointCloud::kXYZs | PointCloud::kNormals | PointCloud::kColors |
          PointCloud::kFeatures;
  if (c <= 0 || c > full_mask) {
    throw std::runtime_error("Invalid CapabilitySet");
  }
}

PointCloud::CapabilitySet ResolveCapabilities(
    const PointCloud& other, PointCloud::CapabilitySet capabilities) {
  if (capabilities == PointCloud::kInherit) {
    return other.capabilities();
  } else {
    return capabilities;
  }
}

// If kExtraInherit is used, then we take on the other's type is used.
// If another extra is used, and we wish to copy the other's extras,
// ensure they are compatible.
ExtraType ResolveFeatureType(const PointCloud& other,
                               PointCloud::CapabilitySet capabilities,
                               const ExtraType& feature_type) {
  if (feature_type == kExtraInherit) {
    return other.extra_type();
  } else {
    if (capabilities & PointCloud::kFeatures) {
      DRAKE_DEMAND(feature_type == other.extra_type());
    }
    return feature_type;
  }
}

// Implements the rules set forth in `SetFrom`.
// @pre Valid point clouds `a` and `b`.
// @post The returned capabilities will be valid for both point clouds. If
//   `c` enables extras, then the extra type will be shared by both point
//   clouds.
PointCloud::CapabilitySet ResolveCapabilities(
    const PointCloud& a,
    const PointCloud& b,
    PointCloud::CapabilitySet capabilities) {
  if (capabilities == PointCloud::kInherit) {
    // If we do not permit a subset, expect the exact same capabilities.
    a.RequireExactCapabilities(b.capabilities(), b.extra_type());
    return a.capabilities();
  } else {
    ExtraType f = (capabilities & PointCloud::kFeatures) ? a.extra_type()
                                                           : kExtraNone;
    a.RequireCapabilities(capabilities, f);
    b.RequireCapabilities(capabilities, f);
    return capabilities;
  }
}

}  // namespace

std::string ToString(PointCloud::CapabilitySet c, const ExtraType& f) {
  return "(" + join(ToStringVector(c, f), " | ") + ")";
}

PointCloud::PointCloud(
    PointCloud::Index new_size,
    PointCloud::CapabilitySet capabilities,
    const ExtraType& feature_type)
    : size_(new_size),
      capabilities_(capabilities),
      extra_type_(feature_type) {
  ValidateCapabilities(capabilities_);
  if (capabilities_ & PointCloud::kInherit)
    throw std::runtime_error("Cannot construct a PointCloud with kInherit");
  if (feature_type == kExtraInherit)
    throw std::runtime_error(
        "Cannot construct a PointCloud with kExtraInherit");
  if (has_extras()) {
    if (extra_type_ == kExtraNone)
      throw std::runtime_error("Cannot specify kExtraNone with kFeatures");
  }
  else if (feature_type != kExtraNone)
    throw std::runtime_error(
        "Must specify kExtraNone if kFeatures is not present");

  storage_.reset(new Storage(new_size, capabilities, feature_type));
  SetDefault(0, size_);
}

PointCloud::PointCloud(const PointCloud& other,
                       PointCloud::CapabilitySet copy_capabilities,
                       const ExtraType& feature_type)
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
  if (has_extras()) {
    set(mutable_extras(), kDefaultValue);
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
  if (c_final & kXYZs) {
    mutable_xyzs() = other.xyzs();
  }
  if (c_final & kColors) {
    mutable_colors() = other.colors();
  }
  if (c_final & kNormals) {
    mutable_normals() = other.normals();
  }
  if (c_final & kFeatures) {
    mutable_extras() = other.extras();
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
  if (c & kXYZs) {
    fresh_block(mutable_xyzs()) = other.xyzs();
  }
  if (c & kNormals) {
    fresh_block(mutable_normals()) = other.normals();
  }
  if (c & kColors) {
    fresh_block(mutable_colors()) = other.colors();
  }
  if (c & kFeatures) {
    fresh_block(mutable_extras()) = other.extras();
  }
}

bool PointCloud::has_xyzs() const {
  return capabilities_ & kXYZs;
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
  return capabilities_ & kColors;
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
  return capabilities_ & kNormals;
}
Eigen::Ref<const Matrix3X<T>> PointCloud::normals() const {
  DRAKE_DEMAND(has_normals());
  return storage_->normals();
}
Eigen::Ref<Matrix3X<T>> PointCloud::mutable_normals() {
  DRAKE_DEMAND(has_normals());
  return storage_->normals();
}

bool PointCloud::has_extras() const {
  return capabilities_ & kFeatures;
}
bool PointCloud::has_extras(const ExtraType& feature_type) const {
  return has_extras() && extra_type_ == feature_type;
}
Eigen::Ref<const MatrixX<F>> PointCloud::extras() const {
  DRAKE_DEMAND(has_extras());
  return storage_->features();
}
Eigen::Ref<MatrixX<F>> PointCloud::mutable_extras() {
  DRAKE_DEMAND(has_extras());
  return storage_->features();
}

bool PointCloud::HasCapabilities(
    PointCloud::CapabilitySet c,
    const ExtraType& f) const {
  bool good = true;
  if ((capabilities() & c) != c) {
    good = false;
  } else {
    if (c & PointCloud::kFeatures) {
      DRAKE_DEMAND(f != kExtraNone);
      DRAKE_DEMAND(f != kExtraInherit);
      if (extra_type() != f) {
        good = false;
      }
    } else {
      DRAKE_DEMAND(f == kExtraNone);
    }
  }
  return good;
}

void PointCloud::RequireCapabilities(
    CapabilitySet c,
    const ExtraType& f) const {
  if (!HasCapabilities(c, f)) {
    throw std::runtime_error(
        fmt::format("PointCloud does not have expected capabilities.\n"
                    "Expected {}, got {}",
                    ToString(c, f),
                    ToString(capabilities(), extra_type())));
  }
}

bool PointCloud::HasExactCapabilities(
      CapabilitySet c,
      const ExtraType& f) const {
  return HasCapabilities(c, f) && capabilities() == c;
}

void PointCloud::RequireExactCapabilities(
    CapabilitySet c,
    const ExtraType& f) const {
  if (!HasExactCapabilities(c, f)) {
    throw std::runtime_error(
        fmt::format("PointCloud does not have the exact expected capabilities."
                    "\nExpected {}, got {}",
                    ToString(c, f),
                    ToString(capabilities(), extra_type())));
  }
}

}  // namespace perception
}  // namespace drake

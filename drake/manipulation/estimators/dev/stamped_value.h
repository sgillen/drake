#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {

/**
 * Primitive stamp, using a double.
 **/
template <typename T = double>
class Stamp {
 public:
  explicit Stamp(const T& value)
    : value_(value) {}
  Stamp& operator=(const T& value) {
    value_ = value;
  }
  T& value() { return value_; }
  const T& value() const { return value_; }

  static Stamp invalid_value() {
    return Stamp(std::numeric_limits<double>::quiet_NaN());
  }
  bool is_valid() const {
    return !std::isnan(value_);
  }
 private:
  T value_{};
};

/**
 * A stamped value, where the stamp could be a floating-point timestamp, or
 * could use some other type.
 */
template <typename T, typename S = double>
class StampedValue {
 public:
  using StampType = Stamp<S>;

  StampedValue()
      : stamp_(StampType::invalid_value()) {}
  StampedValue(const StampType& stamp, const T& value)
      : stamp_(stamp), value_(value) {}
  const StampType& stamp() const { return stamp_; }
  const T& value() const { return value_; }
  bool has_data() const { return stamp_.is_valid(); }
  T& mutable_value(const StampType& new_stamp) {
    stamp_ = new_stamp;
    return value_;
  }
  // Implicit cast, for use inside of an Eigen matrix
  operator S() const { return stamp_; }
  operator T() const { return value_; }
 private:
  S stamp_{};
  T value_{};
};

/**
 * A simple vector of stamped values.
 * \code
 *    VectorStampedXd stamped_value;
 *    auto value = stamped_value.cast<double>();  // Get values.
 *    auto stamps = stamped_value.cast<Stamp<>>();  // Get stamps.
 * \code
 */
template <typename T>
using VectorStampedX = VectorX<StampedValue<T>>;

using VectorStampedXd = VectorStampedX<double>;

}  // manipulation
}  // drake

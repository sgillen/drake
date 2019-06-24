#pragma once

#include "drake/common/drake_deprecated.h"

namespace drake {
namespace example_class {

/// Example class.
class ExampleCppClass {
 public:
  /// Good constructor.
  ExampleCppClass() {}

  DRAKE_DEPRECATED("2038-01-19", "Do not use ExampleCppClass(int)")
  explicit ExampleCppClass(int) {}

  DRAKE_DEPRECATED("2038-01-19", "Do not use ExampleCppClass(double)")
  explicit ExampleCppClass(double) {}

  DRAKE_DEPRECATED("2038-01-19", "Do not use DeprecatedMethod()")
  void DeprecatedMethod() {}

  DRAKE_DEPRECATED("2038-01-19", "Do not use deprecated_prop")
  int deprecated_prop{};

  /// Good overload.
  void overload() {}

  DRAKE_DEPRECATED("2038-01-19", "Do not use overload(int)")
  void overload(int) {}
};

}  // namespace example_class
}  // namespace drake

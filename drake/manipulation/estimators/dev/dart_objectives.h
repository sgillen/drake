#pragma once

#include "drake/manipulation/estimators/dev/dart.h"

namespace drake {
namespace manipulation {

class DartJointObjective : public DartObjective {
 public:
  void Observe(const KinematicsState& full_state) {
  }
 private:
};

}  // namespace manipulation
}  // namespace drake

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/text_logging_gflags.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace push_and_pick {

class PerceptionBase;

int DoMain(std::unique_ptr<PerceptionBase> perception = nullptr);

}  // namespace push_and_pick
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

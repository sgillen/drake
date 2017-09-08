#include <gflags/gflags.h>

#include "drake/common/text_logging_gflags.h"

#include "drake/examples/kuka_iiwa_arm/push_pick_place/push_and_pick_demo.h"

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::kuka_iiwa_arm::push_and_pick::DoMain();
}

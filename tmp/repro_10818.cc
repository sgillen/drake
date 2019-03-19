#include "drake/common/text_logging.h"
#include "drake/examples/pendulum/pendulum_plant.h"

namespace drake {
namespace {

using examples::pendulum::PendulumPlant;

int DoMain() {
  drake::log()->set_level(spdlog::level::debug);

  PendulumPlant<double> plant;
  auto context = plant.CreateDefaultContext();
  auto derivs = plant.AllocateTimeDerivatives();
  plant.CalcTimeDerivatives(*context, derivs.get());
  drake::log()->info("Done");
  return 0;
}

}
}

int main() {
  return drake::DoMain();
}

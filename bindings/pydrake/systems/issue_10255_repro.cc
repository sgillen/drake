#include <gtest/gtest.h>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace systems {

using T = double;

class DemoSystem : public LeafSystem<T> {
 public:
  DemoSystem() : LeafSystem<T>() {
    const double period_sec = 1.;
    const double offset_sec = 0.;
    DeclarePeriodicDiscreteUpdate(period_sec, offset_sec);
  }

  int num_updates() const { return num_updates_; }

 protected:

  void DoCalcDiscreteVariableUpdates(const Context<T>&,
        const std::vector<const DiscreteUpdateEvent<T>*>&,
        DiscreteValues<T>*) const override {
    ++num_updates_;
  }

 private:
  mutable int num_updates_{0};
};

GTEST_TEST(Blarg, Blarg) {
  drake::log()->info("nextafter 0: {}", std::nextafter(0., 1.));
  DemoSystem system;
  Simulator<T> simulator(system);
  simulator.StepTo(0.);
  EXPECT_EQ(system.num_updates(), 1);
}

}
}

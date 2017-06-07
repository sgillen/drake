// Testing to see how frequently publish gets called...
#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/diagram_builder.h"

#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace systems {
namespace {

using namespace std;

const double dt = 0.1;
typedef double T;

typedef std::function<void(const string&, double)> TimeCallback;

class StepObserver : public LeafSystem<T> {
 public:
  StepObserver(TimeCallback callback)
      : callback_(callback) {
    DeclarePerStepAction(DiscreteEvent<T>::kDiscreteUpdateAction);
  }
  void DoCalcOutput(
        const Context<T>& context,
        SystemOutput<T>*) const override {
    callback_("Step::CalcOutput", context.get_time());
  }
  void DoCalcDiscreteVariableUpdates(
      const Context<T>& context, DiscreteValues<T>*) const override {
    callback_("Step::DiscreteUpdate", context.get_time());
  }
 private:
  TimeCallback callback_;
};

class PeriodicUpdater : public LeafSystem<T> {
 public:
  PeriodicUpdater(TimeCallback callback)
      : callback_(callback) {
    DeclarePublishPeriodSec(dt);
    DeclarePeriodicUnrestrictedUpdate(dt, 0);
    DeclarePeriodicDiscreteUpdate(dt, 0);
  }
  void DoCalcOutput(const Context<T>& context, SystemOutput<T>*) const override {
    callback_("Periodic::CalcOutput", context.get_time());
  }
  void DoPublish(const Context<T>& context) const override {
    callback_("Periodic::Publish", context.get_time());
  }
  void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      DiscreteValues<T>*) const override {
    callback_("Periodic::DiscreteUpdate", context.get_time());
  }
  void DoCalcUnrestrictedUpdate(const Context<T>& context,
                                State<T>*) const override {
    callback_("Periodic::UnrestrictedUpdate", context.get_time());
  }
 private:
  TimeCallback callback_;
};

// Tests that DeclareAbstractState works expectedly.
GTEST_TEST(ModelLeafSystemTest, ModelAbstractState) {
  DiagramBuilder<T> builder;

  map<string, vector<double>> times;
  auto callback = [&times](const string& type, double time) {
    auto& list = times[type];
    cout
        << "[" << list.size() << "] "
        << type << ": " << time << endl;
    list.push_back(time);
  };

  builder.AddSystem<StepObserver>(callback);
  builder.AddSystem<PeriodicUpdater>(callback);
  auto sys = builder.Build();

  Simulator<T> simulator(*sys);
  simulator.Initialize();
  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);  // Useless until we init the camera properly
  simulator.StepTo(0.2);
}

}  // namespace
}  // namespace systems
}  // namespace drake

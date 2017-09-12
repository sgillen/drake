#include <memory>

#include <Eigen/Dense>

#include <gtest/gtest.h>

#include "drake/common/text_logging.h"
#include "drake/systems/framework/leaf_system.h"

using std::unique_ptr;
using std::make_unique;

namespace drake {
namespace systems {

struct Info {
  std::array<bool, 4> had_value;
};

// Structure:
//  - Vector-Valued: Ports 0 + 1
//  - Abstract-Valued: Ports 2 + 3
class OptionalPortSystem : public LeafSystem<double> {
 public:
  OptionalPortSystem() {
    this->DeclareInputPort(kVectorValued, 1);
    this->DeclareInputPort(kVectorValued, 1);
    this->DeclareAbstractInputPort();
    this->DeclareAbstractInputPort();

    this->DeclareAbstractOutputPort(
        Info(), &OptionalPortSystem::DoCalcOutput);
  }

  void DoCalcOutput(
      const Context<double>& context,
      Info* output) const {
    const BasicVector<double>* v1 = this->EvalVectorInput(context, 0);
    const BasicVector<double>* v2 = this->EvalVectorInput(context, 1);

    const AbstractValue* a1 = this->EvalAbstractInput(context, 2);
    const AbstractValue* a2 = this->EvalAbstractInput(context, 3);

    output->had_value = {{
        v1 != nullptr,
        v2 != nullptr,
        a1 != nullptr,
        a2 != nullptr
    }};
  };
};

GTEST_TEST(OptionalPortTest, CheckIt) {
  auto sys = make_unique<OptionalPortSystem>();
  auto context = sys->CreateDefaultContext();
  auto output = sys->AllocateOutput(*context);

  Eigen::VectorXd value(1);
  value << 0;

  context->FixInputPort(0, value);
  // Skip 1.
  context->FixInputPort(2, AbstractValue::Make<bool>(true));

  sys->CalcOutput(*context, output.get());

  const Info& info = output->get_data(0)->GetValueOrThrow<Info>();

  Info info_expected{{{true, false, true, false}}};
  EXPECT_EQ(info.had_value, info_expected.had_value);
}

}  // namespace systems
}  // namespace drake

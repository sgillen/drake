//
// Created by eacousineau on 4/21/17.
//

#include "cost.h"

namespace drake {
namespace solvers {

void Cost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                  Eigen::VectorXd& y) const {
  impl_->DoEval(x, y);
}
void Cost::DoEval(const Eigen::Ref<const AutoDiffVecXd> &x,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                  AutoDiffVecXd &y) const override {
  impl_->DoEval(x, y);
}

}
}

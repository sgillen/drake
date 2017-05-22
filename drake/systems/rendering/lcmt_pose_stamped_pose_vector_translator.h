#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"

namespace drake {
namespace systems {
namespace rendering {

class LcmtPoseStampedPoseVectorTranslator :
      public lcm::LcmAndVectorBaseTranslator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmtPoseStampedPoseVectorTranslator)

  LcmtPoseStampedPoseVectorTranslator();

  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
                   VectorBase<double>* vector_base) const  override;

  void Serialize(double time, const VectorBase<double>& vector_base,
                 std::vector<uint8_t>* lcm_message_bytes) const override;

  std::unique_ptr<BasicVector<double>> AllocateOutputVector() const override;
};

}  // rendering
}  // systems
}  // drake

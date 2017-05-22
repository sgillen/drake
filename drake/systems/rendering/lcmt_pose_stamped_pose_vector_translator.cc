#include "drake/systems/rendering/lcmt_pose_stamped_pose_vector_translator.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace systems {
namespace rendering {

LcmtPoseStampedPoseVectorTranslator::LcmtPoseStampedPoseVectorTranslator()
    : lcm::LcmAndVectorBaseTranslator(PoseVector<double>::kSize) {
}

void LcmtPoseStampedPoseVectorTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    VectorBase<double>* vector_base) const {
  drake::lcmt_pose_stamped transform_msg;
  transform_msg.decode(lcm_message_bytes, 0, lcm_message_length);

  Eigen::Translation<double, 3> t(
      transform_msg.trans[0],
      transform_msg.trans[1],
      transform_msg.trans[2]);

  Eigen::Quaternion<double> quat(
      transform_msg.quat[0],
      transform_msg.quat[1],
      transform_msg.quat[2],
      transform_msg.quat[3]);

  auto pose_vector = dynamic_cast<PoseVector<double>*>(vector_base);
  pose_vector->set_translation(t);
  pose_vector->set_rotation(quat);
}

void LcmtPoseStampedPoseVectorTranslator::Serialize(
    double time, const VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  drake::lcmt_pose_stamped transform_msg;
  transform_msg.utime = static_cast<int64_t>(time * 1000000);

  std::cout << "serializing" << std::endl;

  auto pose_vector = dynamic_cast<const PoseVector<double>*>(&vector_base);
  const auto t = pose_vector->get_translation();
  transform_msg.trans[0] = t.x();
  transform_msg.trans[1] = t.y();
  transform_msg.trans[2] = t.z();

  // TODO Verify the order of quat element in rigid_transform_t.
  const auto quat = pose_vector->get_rotation();
  transform_msg.quat[0] = quat.w();
  transform_msg.quat[1] = quat.x();
  transform_msg.quat[2] = quat.y();
  transform_msg.quat[3] = quat.z();

  const int kEncodedSize = transform_msg.getEncodedSize();
  lcm_message_bytes->resize(kEncodedSize);
  transform_msg.encode(lcm_message_bytes->data(), 0, kEncodedSize);
}

std::unique_ptr<BasicVector<double>>
LcmtPoseStampedPoseVectorTranslator::AllocateOutputVector() const {
  return std::make_unique<rendering::PoseVector<double>>();
}

}  // rendering
}  // systems
}  // drake

#include "drake/systems/rendering/pose_stamped_t_pose_vector_translator.h"

#include "drake/systems/rendering/pose_vector.h"
#include "robotlocomotion/pose_stamped_t.hpp"

namespace drake {
namespace systems {
namespace rendering {

PoseStampedTPoseVectorTranslator::PoseStampedTPoseVectorTranslator()
    : lcm::LcmAndVectorBaseTranslator(PoseVector<double>::kSize) {
}

void PoseStampedTPoseVectorTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    VectorBase<double>* vector_base) const {
  robotlocomotion::pose_stamped_t pose_msg;
  pose_msg.decode(lcm_message_bytes, 0, lcm_message_length);

  Eigen::Translation<double, 3> t(
      pose_msg.pose.position.x,
      pose_msg.pose.position.y,
      pose_msg.pose.position.z);

  Eigen::Quaterniond quat(
      pose_msg.pose.orientation.w,
      pose_msg.pose.orientation.x,
      pose_msg.pose.orientation.y,
      pose_msg.pose.orientation.z);

  auto pose_vector = dynamic_cast<PoseVector<double>*>(vector_base);
  pose_vector->set_translation(t);
  pose_vector->set_rotation(quat);
}

void PoseStampedTPoseVectorTranslator::Serialize(
    double time, const VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  robotlocomotion::pose_stamped_t pose_msg;
  pose_msg.header.utime = static_cast<int64_t>(time * 1000000);
  pose_msg.header.frame_name = "SHOULD_BE_REPLACED_WITH_RIGHT_ONE";

  auto pose_vector = dynamic_cast<const PoseVector<double>*>(&vector_base);
  const auto t = pose_vector->get_translation();
  pose_msg.pose.position.x = t.x();
  pose_msg.pose.position.y = t.y();
  pose_msg.pose.position.z = t.z();

  const auto quat = pose_vector->get_rotation();
  pose_msg.pose.orientation.w = quat.w();
  pose_msg.pose.orientation.x = quat.x();
  pose_msg.pose.orientation.y = quat.y();
  pose_msg.pose.orientation.z = quat.z();

  const int kEncodedSize = pose_msg.getEncodedSize();
  lcm_message_bytes->resize(kEncodedSize);
  pose_msg.encode(lcm_message_bytes->data(), 0, kEncodedSize);
}

std::unique_ptr<BasicVector<double>>
PoseStampedTPoseVectorTranslator::AllocateOutputVector() const {
  return std::make_unique<rendering::PoseVector<double>>();
}

}  // rendering
}  // systems
}  // drake

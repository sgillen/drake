#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {
namespace sensors {

// TODO rewrite comments here.
/// A ImageToLcmMessage takes as input a Image<T> and the pose of the camera
/// in the world (`X_WC`). If the input port containing `X_WC` is unconnected,
/// a std::runtime_error will be thrown
/// while evaluating the output of this system. This system outputs an
/// AbstractValue containing a `Value<bot_core::image_t>` LCM message that
/// defines an image. This message can then be sent to
/// `drake-visualizer` using LcmPublisherSystem for visualizing the image.


class ImageToLcmMessage : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImageToLcmMessage)

  /// A %ImageToLcmMessage constructor.
  ImageToLcmMessage();

  /// Returns a descriptor of the input port containing a color image.
  const InputPortDescriptor<double>& color_image_input_port() const;

  /// Returns a descriptor of the input port containing a depth image.
  const InputPortDescriptor<double>& depth_image_input_port() const;

  /// Returns a descriptor of the input port containing a label image.
  const InputPortDescriptor<double>& label_image_input_port() const;

  /// Returns a descriptor of the abstract valued output port that contains a
  /// `Value<bot_core::images_t>`.
  const OutputPortDescriptor<double>& images_t_msg_output_port() const;

 protected:
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

 private:
  int color_image_input_port_index_{};
  int depth_image_input_port_index_{};
  int label_image_input_port_index_{};
  int images_t_msg_output_port_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake

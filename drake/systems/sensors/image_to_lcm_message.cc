#include "drake/systems/sensors/image_to_lcm_message.h"

#include <string>
#include <vector>

#include "bot_core/images_t.hpp"
#include "bot_core/image_t.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/image.h"
#include <zlib.h>

namespace drake {
namespace systems {

using rendering::PoseVector;

namespace sensors {
namespace {

template <PixelType kPixelType>
void Compress(const Image<kPixelType>& image, bot_core::image_t* msg) {
  const int original_size = image.size() * sizeof(image.at(0, 0)[0]);
  uint64_t buf_size = original_size * 1.2;
  std::unique_ptr<uint8_t> buf(new uint8_t[buf_size]);

  auto compress_status = compress2(
      buf.get(), &buf_size, reinterpret_cast<const Bytef*>(image.at(0, 0)),
      original_size, Z_BEST_SPEED);

  DRAKE_DEMAND(compress_status == Z_OK);

  msg->data.resize(buf_size);
  msg->size = buf_size;
  memcpy(&msg->data[0], buf.get(), buf_size);
}

template <PixelType kPixelType>
void PackImageToLcmMessage(const Image<kPixelType>& image,
                           bot_core::image_t* msg) {
  msg->width = image.width();
  msg->height = image.height();
  msg->nmetadata = 0;
  msg->row_stride = image.kNumChannels * msg->width * sizeof(image.at(0, 0)[0]);
  msg->pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
  Compress(image, msg);
}

}  // anonymous namespace


ImageToLcmMessage::ImageToLcmMessage() {
  color_image_input_port_index_ =
      DeclareAbstractInputPort(
          systems::Value<ImageRgba8U>()).get_index();

  depth_image_input_port_index_ =
      DeclareAbstractInputPort(
          systems::Value<ImageDepth32F>()).get_index();

  // label_image_input_port_index_ =
  //     DeclareAbstractInputPort(
  //         systems::Value<ImageLabel16I>()).get_index();

  images_t_msg_output_port_index_ =
      DeclareAbstractOutputPort(systems::Value<bot_core::images_t>())
          .get_index();
}

const InputPortDescriptor<double>&
ImageToLcmMessage::color_image_input_port() const {
  return this->get_input_port(color_image_input_port_index_);
}

const InputPortDescriptor<double>&
ImageToLcmMessage::depth_image_input_port() const {
  return this->get_input_port(depth_image_input_port_index_);
}

const InputPortDescriptor<double>&
ImageToLcmMessage::label_image_input_port() const {
  return this->get_input_port(label_image_input_port_index_);
}

const OutputPortDescriptor<double>&
ImageToLcmMessage::images_t_msg_output_port() const {
  return System<double>::get_output_port(images_t_msg_output_port_index_);
}

void ImageToLcmMessage::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {

  const ImageRgba8U& color_image = this->EvalAbstractInput(
      context, color_image_input_port_index_)->GetValue<ImageRgba8U>();

  const ImageDepth32F& depth_image = this->EvalAbstractInput(
      context, depth_image_input_port_index_)->GetValue<ImageDepth32F>();

  // const ImageLabel16I& label_image = this->EvalAbstractInput(
  //     context, label_image_input_port_index_)->GetValue<ImageLabel16I>();

  bot_core::image_t color_image_msg;
  PackImageToLcmMessage(color_image, &color_image_msg);

  bot_core::image_t depth_image_msg;
  PackImageToLcmMessage(depth_image, &depth_image_msg);

  // bot_core::image_t label_image_msg;
  // PackImageToLcmMessage(label_image, &label_image_msg);

  bot_core::images_t& msg =
      output->GetMutableData(images_t_msg_output_port_index_)->
          GetMutableValue<bot_core::images_t>();

  msg.utime = static_cast<int64_t>(context.get_time() * 1000000);
  msg.n_images = 2;

  msg.image_types.clear();
  // To be comatible with Director except label image.
  msg.image_types.push_back(bot_core::images_t::LEFT);
  msg.image_types.push_back(bot_core::images_t::DEPTH_MM_ZIPPED);  // 6
  msg.image_types.push_back(bot_core::images_t::MASK_ZIPPED);  // 3

  msg.images.clear();
  msg.images.push_back(color_image_msg);
  msg.images.push_back(depth_image_msg);
  // msg.images.push_back(label_image_msg);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake

const int16_t bot_core::images_t::LEFT;
const int16_t bot_core::images_t::DEPTH_MM_ZIPPED;
const int16_t bot_core::images_t::MASK_ZIPPED;

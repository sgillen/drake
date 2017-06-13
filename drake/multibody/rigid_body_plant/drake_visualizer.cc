#include "drake/multibody/rigid_body_plant/drake_visualizer.h"

#include <chrono>
#include <thread>
#include <utility>

#include "drake/common/text_logging.h"
#include "drake/multibody/rigid_body_plant/create_load_robot_message.h"
#include "drake/systems/rendering/drake_visualizer_client.h"

namespace drake {
namespace systems {

namespace {
// Defines the index of the port that the DrakeVisualizer uses.
const int kPortIndex = 0;
}  // namespace

DrakeVisualizer::DrakeVisualizer(const RigidBodyTree<double>& tree,
                                 drake::lcm::DrakeLcmInterface* lcm,
                                 bool enable_playback,
                                 const std::string& prefix)
    : lcm_(lcm),
      load_message_(multibody::CreateLoadRobotMessage<double>(tree)),
      draw_message_translator_(tree),
      prefix_(prefix) {
  set_name("drake_visualizer");
  const int vector_size =
      tree.get_num_positions() + tree.get_num_velocities();
  DeclareInputPort(kVectorValued, vector_size);
  this->DeclareDiscreteState(1);
  DeclarePerStepAction(DiscreteEvent<double>::kPublishAction);
  if (enable_playback) log_.reset(new SignalLog<double>(vector_size));
}

void DrakeVisualizer::set_publish_period(double period) {
  LeafSystem<double>::DeclarePublishPeriodSec(period);
}

void DrakeVisualizer::DoCalcNextUpdateTime(
    const Context<double>& context, UpdateActions<double>* events) const {
  if (is_load_message_sent(context)) {
    return LeafSystem<double>::DoCalcNextUpdateTime(context, events);
  } else {
    // TODO(siyuan): cleanup after #5725 is resolved.
    events->time = context.get_time() + 0.0001;
    DiscreteEvent<double> event;
    event.action = DiscreteEvent<double>::ActionType::kDiscreteUpdateAction;
    events->events.push_back(event);
  }
}

void DrakeVisualizer::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  DRAKE_DEMAND(!is_load_message_sent(context));

  PublishLoadRobot();
  set_is_load_message_sent(discrete_state, true);
}

void DrakeVisualizer::ReplayCachedSimulation() const {
  if (log_ != nullptr) {
    PlaybackTrajectory(GetReplayCachedSimulation());
  } else {
    drake::log()->warn(
        "DrakeVisualizer::ReplayCachedSimulation() called on instance that "
        "wasn't initialized to record. Next time, please construct "
        "DrakeVisualizer with recording enabled.");
  }
}

PiecewisePolynomial<double> DrakeVisualizer::GetReplayCachedSimulation() const
{
  DRAKE_ASSERT(log_ != nullptr);
  // Build piecewise polynomial
  auto times = log_->sample_times();
  // NOTE: The SignalLog can record signal for multiple identical time stamps.
  //  This culls the duplicates as required by the PiecewisePolynomial.
  std::vector<int> included_times;
  included_times.reserve(times.rows());
  std::vector<double> breaks;
  included_times.push_back(0);
  breaks.push_back(times(0));
  int last = 0;
  for (int i = 1; i < times.rows(); ++i) {
    double val = times(i);
    if (val != breaks[last]) {
      breaks.push_back(val);
      included_times.push_back(i);
      ++last;
    }
  }

  auto sample_data = log_->data();
  std::vector<MatrixX<double>> knots;
  knots.reserve(sample_data.cols());
  for (int c : included_times) {
    knots.push_back(sample_data.col(c));
  }
  return PiecewisePolynomial<double>::ZeroOrderHold(breaks, knots);
}

void DrakeVisualizer::PlaybackTrajectoryFrame(
    const PiecewisePolynomial<double>& input_trajectory,
    double sim_time) const {
  // TODO(eric.cousineau): Consider caching this elsewhere?
  DRAKE_ASSERT(sim_time >= 0);
  if (sim_time > input_trajectory.getEndTime()) {
    drake::log()->warn("Skipping out-of-bounds frame: {}", sim_time);
  } else {
    BasicVector<double> data(log_->get_input_size());
    data.set_value(input_trajectory.value(sim_time));

    // Translates the input vector into an array of bytes representing an LCM
    // message.
    std::vector<uint8_t> message_bytes;
    draw_message_translator_.Serialize(sim_time, data,
                                       &message_bytes);

    // Publishes onto the specified LCM channel.
    lcm_->Publish(prefix_ + "DRAKE_VIEWER_DRAW", message_bytes.data(),
                  message_bytes.size());
  }
}

void DrakeVisualizer::PlaybackTrajectory(
    const PiecewisePolynomial<double>& input_trajectory) const {
  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::duration<double>;
  using TimePoint = std::chrono::time_point<Clock, Duration>;

  // Target frame length at 60 Hz playback rate.
  const double kFrameLength = 1 / 60.0;
  double sim_time = input_trajectory.getStartTime();
  TimePoint prev_time = Clock::now();

  while (sim_time < input_trajectory.getEndTime()) {
    PlaybackTrajectoryFrame(input_trajectory, sim_time);
    const TimePoint earliest_next_frame = prev_time + Duration(kFrameLength);
    std::this_thread::sleep_until(earliest_next_frame);
    TimePoint curr_time = Clock::now();
    sim_time += (curr_time - prev_time).count();
    prev_time = curr_time;
  }

  // Final evaluation is at the final time stamp, guaranteeing the final state
  // is visualized.
  PlaybackTrajectoryFrame(input_trajectory, input_trajectory.getEndTime());
}

void DrakeVisualizer::DoPublish(const Context<double>& context) const {
  if (!is_load_message_sent(context)) {
    drake::log()->warn(
        "DrakeVisualizer::Publish() called before PublishLoadRobot()");
    return;
  }

  // Obtains the input vector, which contains the generalized q,v state of the
  // RigidBodyTree.
  const BasicVector<double>* input_vector = EvalVectorInput(context,
                                                            kPortIndex);
  if (log_ != nullptr) {
    bool skip_entry = false;
    double cur_time = context.get_time();
    if (log_->sample_times().rows() > 0) {
      // TODO(eric.cousineau): Encountering very small timesteps in simulator.
      // Need to figure out if this is due to my publishing setup.
      double prev_time = log_->sample_times().tail(1)(0);
      DRAKE_ASSERT(cur_time > prev_time);
      const double kEpsilon = 1e-10;
      double diff = cur_time - prev_time;
      if (diff < kEpsilon) {
        drake::log()->error("At sim_time = {}, encountered small timestep {}."
                           " Skipping",
                           cur_time, diff);
        skip_entry = true;
      }
    }
    if (!skip_entry) {
      log_->AddData(cur_time, input_vector->get_value());
    }
  }

  // Translates the input vector into an array of bytes representing an LCM
  // message.
  std::vector<uint8_t> message_bytes;
  draw_message_translator_.Serialize(context.get_time(), *input_vector,
                                     &message_bytes);

  // Publishes onto the specified LCM channel.
  lcm_->Publish(prefix_ + "DRAKE_VIEWER_DRAW", message_bytes.data(),
                message_bytes.size());
}

void DrakeVisualizer::PublishLoadRobot() const {
  const int lcm_message_length = load_message_.getEncodedSize();
  std::vector<uint8_t> lcm_message_bytes{};
  lcm_message_bytes.resize(lcm_message_length);
  load_message_.encode(lcm_message_bytes.data(), 0, lcm_message_length);

  lcm_->Publish(prefix_ + "DRAKE_VIEWER_LOAD_ROBOT", lcm_message_bytes.data(),
      lcm_message_length);
}

}  // namespace systems
}  // namespace drake

#include "drake/manipulation/estimators/dev/articulated_state_estimator.h"

#include <string>
#include <yaml-cpp/yaml.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/manipulation/estimators/dev/ManipulationTrackerLoader.hpp"

#include "drake/manipulation/estimators/dev/tree_state_portion.h"

using namespace std;

namespace drake {
namespace manipulation {

//typedef double T;
//typedef systems::Context<T> Context;
//typedef systems::DiscreteValues<T> DiscreteValues;
//typedef systems::SystemOutput<T> SystemOutput;
//using systems::Value;
//using Inport = systems::InputPortDescriptor<T>;
//using Outport = systems::OutputPortDescriptor<T>;

using Eigen::VectorXd;
typedef Eigen::Matrix3Xd PointCloud;
typedef systems::sensors::ImageDepth32F DepthImage;

class ArticulatedStateEstimator::Impl {
 public:
  using StatePortion = KinematicStatePortion<T>;

  Impl(ArticulatedStateEstimator* system,
       const string& config_file)
      : system_(system) {
    lcm_.reset(new ::lcm::LCM());
    string drc_path = "";
    auto config = YAML::LoadFile(config_file);
    loader_.reset(new ManipulationTrackerLoader(config, drc_path, lcm_));

    const int num_positions = world_robot().get_num_positions();

    param_initial_condition_index_ = system->DeclareNumericParameter(
          systems::BasicVector<double>(num_positions));

    // Define ports
    inport_point_cloud_
        = &system->DeclareAbstractInputPort(Value<PointCloud>());
    // TODO(eric.cousineau): Make depth image optional?
    inport_depth_image_
        = &system->DeclareAbstractInputPort(Value<DepthImage>());
    // TODO(eric.cousineau): Incorporate something akin to
    // KinematicStatePortion.
    inport_world_state_ =
        &system->DeclareInputPort(systems::kVectorValued, num_positions);
    outport_estimated_world_state_ =
        &system->DeclareOutputPort(systems::kVectorValued, num_positions);

    state_estimated_world_state_index_ = 0;
    system->DeclareDiscreteState(num_positions);
  }

  void DoCalcDiscreteVariableUpdates(const Context& context,
                                     DiscreteValues* updates) const {
    // TODO(eric.cousineau): Dunno, ensure this is only executed once.
    const auto in_world_state = system->EvalVectorInput(context,
                                                        input_world_state_);

    // Update individual costs.
    for (auto& cost : loader_->joint_state_costs_) {
      cost->handleJointStateMsg(in_world_state);
    }
    for (auto& cost : loader_->kinect_frame_costs_) {
      cost->handleKinectFrameMsg(...);
      cost->handleDepthImageMsg(...);
    }
    for (auto& cost : loader_->nonpentrating_costs_) {
      cost->handleSceneState(...);
    }
    estimator().update();
    estimator().publish();

    updates->get_mutable_vector()->get_mutable_value() = estimator().getMean();
  }

  void DoCalcOutput(const Context& context, SystemOutput* output) const {
    const auto estimated_world_state =
        context.get_discrete_state(state_estimated_world_state_index_)
        ->CopyToVector();
    output->GetMutableVectorData(outport_estimated_world_state_->get_index())
        ->get_mutable_value() = estimated_world_state;
  }

 private:
  // Consider relaxing this???
  friend class ArticulatedStateEstimator;

  shared_ptr<::lcm::LCM> lcm_; // HACK, to satisfy API
  shared_ptr<ManipulationTrackerLoader> loader_;
  ManipulationTracker& estimator() const { return *loader_->estimator_; }
  const RigidBodyTreed& world_robot() { return *estimator().getRobot(); }
  // Latest
  PointCloud point_cloud_;
  const Inport* inport_point_cloud_{};
  DepthImage depth_image_;
  const Inport* inport_depth_image_{};
  VectorXd world_state_;
  const Inport* inport_world_state_{};

  ArticulatedStateEstimator* system_{};

  int state_estimated_world_state_index_{};
  int param_initial_condition_index_{};
  const Outport* outport_estimated_world_state_{};
};

ArticulatedStateEstimator::ArticulatedStateEstimator(
    const string& config_file) {
  impl_.reset(new Impl(this, config_file));
}

const ArticulatedStateEstimator::Inport& ArticulatedStateEstimator::inport_point_cloud() const {
  return *impl_->inport_point_cloud_;
}

const ArticulatedStateEstimator::Inport& ArticulatedStateEstimator::inport_depth_image() const
{
  return *impl_->inport_depth_image_;
}

const ArticulatedStateEstimator::Inport& ArticulatedStateEstimator::inport_world_state() const
{
  return *impl_->inport_world_state_;
}

const ArticulatedStateEstimator::Outport& ArticulatedStateEstimator::outport_estimated_world_state() const
{
  return *impl_->outport_estimated_world_state_;
}

}  // namespace manipulation
}  // namespace drake

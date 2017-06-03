#include "drake/manipulation/estimators/dev/articulated_state_estimator.h"

#include <string>
#include <yaml-cpp/yaml.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/manipulation/estimators/dev/ManipulationTrackerLoader.hpp"

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
  Impl(ArticulatedStateEstimator* system,
       const string& config_file) {
    lcm_.reset(new ::lcm::LCM());
    string drc_path = "";
    auto config = YAML::LoadFile(config_file);
    loader_.reset(new ManipulationTrackerLoader(config, drc_path, lcm_));

    // Define ports
    inport_point_cloud_
        = &system->DeclareAbstractInputPort(Value<PointCloud>());
    inport_depth_image_
        = &system->DeclareAbstractInputPort(Value<DepthImage>());
    inport_world_state_
        = &system->DeclareAbstractInputPort(Value<TreeStatePortion>());
    outport_world_state_
        = &system->DeclareAbstractOutputPort(Value<TreeStatePortion>());
  }

  void DoCalcOutput(const Context& context, SystemOutput* output) const {
    context
  }

 private:
  // Consider relaxing this???
  friend class ArticulatedStateEstimator;

  shared_ptr<::lcm::LCM> lcm_; // HACK, to satisfy API
  shared_ptr<ManipulationTrackerLoader> loader_;
  // Latest
  StampedValue<PointCloud> point_cloud_;
  const Inport* inport_point_cloud_{};
  StampedValue<DepthImage> depth_image_;
  const Inport* inport_depth_image_{};
  StampedValue<VectorXd> world_state_;
  const Inport* inport_world_state_{};
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




}  // namespace manipulation
}  // namespace drake

#include "drake/manipulation/estimators/dev/articulated_state_estimator.h"

#include <string>
#include <yaml-cpp/yaml.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_path.h"

#include "drake/manipulation/estimators/dev/ManipulationTrackerLoader.hpp"

#include "drake/manipulation/estimators/dev/tree_state_portion.h"

#include "drake/common/scoped_timer.h"
#include "drake/common/unused.h"

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
  Impl(const string& config_file, const CameraInfo* camera_info) {
    lcm_.reset(new ::lcm::LCM());
    // TODO(eric.cousineau): Use FindResource within the ManipulationTracker code.
    string drc_path = drake::GetDrakePath();
    auto config = YAML::LoadFile(config_file);
    loader_.reset(new ManipulationTrackerLoader(config, drc_path, lcm_,
                                                camera_info));
  }

  void ImplDiscreteUpdate(const VectorXd& q0,
                          const VectorXd& tree_q_measurement,
                          const PointCloud& point_cloud,
                          const DepthImage& depth_image,
                          VectorXd* estimated_tree_state) const {
    ManipulationTracker& estimator = *loader_->estimator_;

    // TODO(eric.cousineau): Is there a better place for this?
    if (!estimator.has_performed_first_update()) {
      estimator.set_q0(q0);
    }

    // Update individual costs.
    for (auto& cost : loader_->joint_state_costs_) {
      // TODO(eric.cousineau): Replace this with only a partial update of the
      // state.
      cost->readTreeState(tree_q_measurement);
    }
    for (auto& cost : loader_->kinect_frame_costs_) {
      cost->readDepthImageAndPointCloud(depth_image, point_cloud);
    }

    // DynamicsCost and NonpenetratingCost presently only rely on priors.
    estimator.update();
    estimator.publish();
    *estimated_tree_state = estimator.getMean();
  }

 private:
  shared_ptr<::lcm::LCM> lcm_; // HACK, to satisfy API
  shared_ptr<ManipulationTrackerLoader> loader_;

  friend class ArticulatedStateEstimator;
};


ArticulatedStateEstimator::ArticulatedStateEstimator(const string& config_file,
                                                     const CameraInfo* camera_info) {
  impl_.reset(new Impl(config_file, camera_info));

  auto& estimator = *impl_->loader_->estimator_;
  auto& scene_tree = *estimator.getRobot();
  const int num_positions = scene_tree.get_num_positions();

  // TODO(eric.cousineau): Just place this in the constructor?
  param_q0_index_ = DeclareNumericParameter(
        systems::BasicVector<double>(estimator.get_q0()));

  // Define ports
  inport_point_cloud_index_ =
      DeclareAbstractInputPort(Value<PointCloud>()).get_index();
  // TODO(eric.cousineau): Make depth image optional?
  inport_depth_image_index_ =
      DeclareAbstractInputPort(Value<DepthImage>()).get_index();
  // TODO(eric.cousineau): Incorporate something akin to
  // KinematicStatePortion.
  inport_tree_q_measurement_index_ =
      DeclareInputPort(systems::kVectorValued, num_positions).get_index();
  outport_tree_state_estimate_index_ =
      DeclareOutputPort(systems::kVectorValued, num_positions).get_index();

  state_tree_state_estimate_index_ = 0;
  DeclareDiscreteState(num_positions);
}

void ArticulatedStateEstimator::DoCalcDiscreteVariableUpdates(
    const Context& context, DiscreteValues* updates) const {
  // TODO(eric.cousineau): Make "loader" an abstract state?
  // TODO(eric.cousineau): Dunno, ensure this is only executed once.
  timing::ScopedWithTimer<> timer("Articulated Update");
  unused(timer);

  std::cout << "Articulated Update Start..." << std::endl;

  // TODO(eric.cousineau): Only get a partial update of the state.
  // Use some projection matrix, or just use indexing?
  const auto tree_q_measurement =
      EvalVectorInput(context, inport_tree_q_measurement_index_)
             ->get_value();
  auto&& point_cloud =
      EvalAbstractInput(context, inport_point_cloud_index_)
             ->GetValue<PointCloud>();
  auto&& depth_image =
      EvalAbstractInput(context, inport_depth_image_index_)
             ->GetValue<DepthImage>();

  VectorXd q0 =
      GetNumericParameter(context, param_q0_index_)
             .get_value();

  VectorXd estimated_tree_state;
  impl_->ImplDiscreteUpdate(q0, tree_q_measurement,
                            point_cloud, depth_image, &estimated_tree_state);
  updates->get_mutable_vector()->get_mutable_value() = estimated_tree_state;
}

void ArticulatedStateEstimator::DoCalcOutput(
    const Context& context, SystemOutput* output) const {
  const auto estimated_world_state =
      context.get_discrete_state(state_tree_state_estimate_index_)
      ->CopyToVector();
  output->GetMutableVectorData(outport_tree_state_estimate_index_)
      ->get_mutable_value() = estimated_world_state;
}

const ArticulatedStateEstimator::Inport& ArticulatedStateEstimator::inport_point_cloud() const {
  return get_input_port(inport_point_cloud_index_);
}

const ArticulatedStateEstimator::Inport& ArticulatedStateEstimator::inport_depth_image() const
{
  return get_input_port(inport_depth_image_index_);
}

const ArticulatedStateEstimator::Inport& ArticulatedStateEstimator::inport_tree_q_measurement() const
{
  return get_input_port(inport_tree_q_measurement_index_);
}

const ArticulatedStateEstimator::Outport& ArticulatedStateEstimator::outport_tree_state_estimate() const
{
  return get_output_port(outport_tree_state_estimate_index_);
}

}  // namespace manipulation
}  // namespace drake

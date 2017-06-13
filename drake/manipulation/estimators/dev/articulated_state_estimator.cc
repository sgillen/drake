#include "drake/manipulation/estimators/dev/articulated_state_estimator.h"

#include <string>
#include <yaml-cpp/yaml.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_path.h"

#include "drake/manipulation/estimators/dev/ManipulationTrackerLoader.hpp"
#include "drake/systems/rendering/pose_vector.h"

#include "drake/manipulation/estimators/dev/vector_slice.h"

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
using systems::rendering::PoseVector;
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

  typedef VectorSlice Slice;
  typedef Slice::Indices Indices;

  void SetQSlices(const Slice& input_slice,
                  const Slice& update_slice) {
    slices_.reset(new Slices {input_slice, update_slice});
  }

  int get_num_input_positions() const {
    return slices_->input.super_size();
  }
  int get_num_update_positions() const {
    return slices_->update.size();
  }
  int get_num_output_positions() const {
    auto robot = loader_->estimator_->getRobot();
    return robot->get_num_positions() + robot->get_num_velocities();
  }

  void ImplDiscreteUpdate(const VectorXd& q0,
                          const VectorXd& tree_q_sys,
                          const Eigen::Isometry3d& X_WD,
                          const PointCloud& point_cloud,
                          const DepthImage& depth_image,
                          VectorXd* estimated_tree_state) const {
    ManipulationTracker& estimator = *loader_->estimator_;

    DRAKE_ASSERT(slices_ != nullptr);

    VectorXd tree_q_sub_common(get_num_update_positions());
    slices_->input.ReadFromSuperset(tree_q_sys,
                                    tree_q_sub_common);
    VectorXd tree_q_est(slices_->update.super_size());
    tree_q_est.setZero();
    slices_->update.WriteToSuperset(tree_q_sub_common,
                                    tree_q_est);

    // TODO(eric.cousineau): Is there a better place for this?
    if (!estimator.has_performed_first_update()) {
      estimator.set_q0(q0);
    }

    // Update individual costs.
    for (auto&& cost : loader_->joint_state_costs_) {
      cost->readTreeState(tree_q_sub_common, slices_->update);
    }
    for (auto&& cost : loader_->robot_state_costs_) {
      cost->readTreeState(tree_q_sub_common, slices_->update);
    }
    for (shared_ptr<KinectFrameCost> cost : loader_->kinect_frame_costs_) {
      cost->readDepthImageAndPointCloud(X_WD, depth_image, point_cloud);
    }

    // DynamicsCost and NonpenetratingCost presently only rely on priors.
    estimator.update(tree_q_est, slices_->update);
    estimator.publish();
    *estimated_tree_state = estimator.getRobotX();
  }

 private:
  shared_ptr<::lcm::LCM> lcm_; // HACK, to satisfy API
  shared_ptr<ManipulationTrackerLoader> loader_;

  struct Slices {
    // Slice from input.
    Slice input;
    // Slice provided to update.
    Slice update;
  };
  shared_ptr<Slices> slices_;

  friend class ArticulatedStateEstimator;
};

ArticulatedStateEstimator::ArticulatedStateEstimator(const string& config_file,
    const CameraInfo* camera_info,
    const std::vector<string>& input_position_names, double dt) {
  impl_.reset(new Impl(config_file, camera_info));

  // This presently does not take in velocities.
  const bool add_velocities = false;
  auto plant_position_names = GetHierarchicalPositionNameList(
      get_tree(), get_plant_id_map(), add_velocities);

  // FIgure out downselection from input port.
  // TODO(eric.cousineau): Figure out how to do this more cleanly at a
  // higher-level. Consider implementing the estimator outside of a system,
  // and feed that and the mapping in, or use something akin to
  // VectorSliceTranslator.
  std::vector<int> plant_indices, input_indices;
  GetCommonIndices(input_position_names, plant_position_names,
                   &input_indices, &plant_indices);
  Impl::Slice input_slice(input_indices, input_position_names.size());
  Impl::Slice update_slice(plant_indices, plant_position_names.size());
  impl_->SetQSlices(input_slice, update_slice);

  auto& estimator = *impl_->loader_->estimator_;

  const int num_input_positions = impl_->get_num_input_positions();
  const int num_output_positions = impl_->get_num_output_positions();

  // TODO(eric.cousineau): Just place this in the constructor?
  param_q0_index_ = DeclareNumericParameter(
        systems::BasicVector<double>(estimator.get_q0()));

  // Define ports
  inport_point_cloud_index_ =
      DeclareAbstractInputPort(Value<PointCloud>()).get_index();
  // TODO(eric.cousineau): Make depth image optional?
  inport_depth_image_index_ =
      DeclareAbstractInputPort(Value<DepthImage>()).get_index();
  inport_depth_camera_pose_index_ =
      DeclareVectorInputPort(PoseVector<double>()).get_index();
  inport_tree_q_measurement_index_ =
      DeclareInputPort(systems::kVectorValued, num_input_positions).get_index();
  outport_tree_state_estimate_index_ =
      DeclareOutputPort(systems::kVectorValued, num_output_positions).get_index();

  state_tree_state_estimate_index_ = 0;
  DeclareDiscreteState(num_output_positions);

  DeclarePeriodicDiscreteUpdate(dt, 0.);
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
  // Ensure that we have the pose from depth camera to the world.
  auto&& X_WD =
      EvalVectorInput<PoseVector>(context, inport_depth_camera_pose_index_)
      ->get_isometry();

  PrintValidPoints(point_cloud, "In system");

  VectorXd q0 =
      GetNumericParameter(context, param_q0_index_)
             .get_value();

  VectorXd estimated_tree_state;
  impl_->ImplDiscreteUpdate(
        q0, tree_q_measurement,
        X_WD, point_cloud, depth_image,
        &estimated_tree_state);
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

const ArticulatedStateEstimator::Inport& ArticulatedStateEstimator::inport_depth_camera_pose() const
{
  return get_input_port(inport_depth_camera_pose_index_);
}

const ArticulatedStateEstimator::Inport& ArticulatedStateEstimator::inport_tree_q_measurement() const
{
  return get_input_port(inport_tree_q_measurement_index_);
}

const ArticulatedStateEstimator::Outport& ArticulatedStateEstimator::outport_tree_state_estimate() const
{
  return get_output_port(outport_tree_state_estimate_index_);
}

const RigidBodyTreed& ArticulatedStateEstimator::get_tree() const
{
  return *impl_->loader_->estimator_->getRobot();
}

const ReverseIdMap& ArticulatedStateEstimator::get_plant_id_map() const
{
  return impl_->loader_->estimator_->getPlantIdMap();
}



}  // namespace manipulation
}  // namespace drake

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

void PrintJointNameHierarchy(const RigidBodyTreed* tree) {
  using std::cout;
  using std::endl;
  int instance_count = tree->get_num_model_instances();
  int position_index = 0;
  for (int instance_id = 0; instance_id < instance_count; ++instance_id) {
    cout << "model[" << instance_id << "]\n";
    auto bodies = tree->FindModelInstanceBodies(instance_id);
    for (const RigidBody<double>* body : bodies) {
      const auto& joint = body->getJoint();
      cout << "  joint: " << joint.get_name() << "\n";
      cout << "  fixed: " << joint.is_fixed() << "\n";
      cout << "  children: \n";
      for (int i = 0; i < joint.get_num_positions(); ++i) {
        cout << "    " << joint.get_position_name(i) << "\n";
        cout << "      flat: " << tree->get_position_name(position_index) << "\n";
        position_index++;
      }
    }
  }
  cout << "flat:\n";
  for (int i = 0; i < tree->get_num_positions(); ++i) {
    cout << "  " << tree->get_position_name(i) << " = " << i << "\n";
  }
  cout << endl;
}

std::vector<std::string> GetHierarchicalPositionNameList(
    const RigidBodyTreed& tree,
    const ReverseIdMap& instance_name_map,
    bool add_velocity) {
  using std::string;
  using std::vector;
  using std::to_string;
  vector<string> names(tree.get_num_positions());
  int instance_count = tree.get_num_model_instances();
  int position_index = 0;
  int velocity_index = 0;
  if (add_velocity) {
    names.resize(tree.get_num_positions() + tree.get_num_velocities());
  }
  for (int instance_id = 0; instance_id < instance_count; ++instance_id) {
    string instance_name = "unknown_" + to_string(instance_id);
    auto iter = instance_name_map.find(instance_id);
    if (iter != instance_name_map.end()) {
      instance_name = iter->second;
    }
    auto bodies = tree.FindModelInstanceBodies(instance_id);
    for (const RigidBody<double>* body : bodies) {
      const auto& joint = body->getJoint();
      for (int i = 0; i < joint.get_num_positions(); ++i) {
        DRAKE_ASSERT(joint.get_position_name(i) == tree.get_position_name(position_index));
        string name = instance_name + "::" + joint.get_position_name(i);
        names[position_index] = name;
        position_index++;
      }
      if (add_velocity) {
        for (int i = 0; i < joint.get_num_velocities(); ++i) {
          DRAKE_ASSERT(joint.get_velocity_name(i) == tree.get_velocity_name(velocity_index));
          string name = instance_name + "::" + joint.get_velocity_name(i);
          names[tree.get_num_positions() + velocity_index] = name;
          velocity_index++;
        }
      }
    }
  }
  return names;
}

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

  typedef VectorSlice<double> Slice;
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
    return loader_->estimator_->getRobot()->get_num_positions();
  }

  void ImplDiscreteUpdate(const VectorXd& q0,
                          const VectorXd& tree_q_measurement_input,
                          const PointCloud& point_cloud,
                          const DepthImage& depth_image,
                          VectorXd* estimated_tree_state) const {
    ManipulationTracker& estimator = *loader_->estimator_;

    DRAKE_ASSERT(slices_ != nullptr);

    VectorXd tree_q_measurement(get_num_update_positions());
    slices_->input.ReadFromSuperset(tree_q_measurement_input,
                                    tree_q_measurement);

    // TODO(eric.cousineau): Is there a better place for this?
    if (!estimator.has_performed_first_update()) {
      estimator.set_q0(q0);
    }

    // Update individual costs.
    for (auto&& cost : loader_->joint_state_costs_) {
      // TODO(eric.cousineau): Replace this with only a partial update of the
      // state.
      cost->readTreeState(tree_q_measurement, slices_->update);
    }
    for (auto& cost : loader_->kinect_frame_costs_) {
      cost->readDepthImageAndPointCloud(depth_image, point_cloud);
    }

    // DynamicsCost and NonpenetratingCost presently only rely on priors.
    estimator.update();
    estimator.publish();
    *estimated_tree_state = estimator.getRobotQ();
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

template <typename T>
std::map<T, int> CreateIndexMap(const std::vector<T> &x) {
  int i = 0;
  std::map<T, int> out;
  for (const auto& value : x) {
    DRAKE_ASSERT(out.find(value) == out.end());
    out[value] = i;
    i++;
  }
  return std::move(out);
}

// TODO(eric.cousineau): Reduce from O(n^2)
template <typename T>
void GetCommonIndices(const std::vector<T> &a,
                      const std::vector<T> &b,
                      std::vector<int>* a_indices,
                      std::vector<int>* b_indices) {
  auto a_map = CreateIndexMap(a);
  auto b_map = CreateIndexMap(b);
  vector<bool> a_found(a.size(), false);
  vector<bool> b_found(b.size(), false);
  for (const auto& a_pair : a_map) {
    auto b_iter = b_map.find(a_pair.first);
    if (b_iter != b_map.end()) {
      auto& b_pair = *b_iter;
      a_indices->push_back(a_pair.second);
      b_indices->push_back(b_pair.second);
      a_found[a_pair.second] = true;
      b_found[b_pair.second] = true;
    }
  }
  cout << "a not found:\n";
  for (int i = 0; i < (int)a.size(); ++i) {
    if (!a_found[i]) {
      cout << "  " << a[i] << endl;
    }
  }
  cout << "b not found:\n";
  for (int i = 0; i < (int)b.size(); ++i) {
    if (!b_found[i]) {
      cout << "  " << b[i] << endl;
    }
  }
}

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

  VectorXd q0 =
      GetNumericParameter(context, param_q0_index_)
             .get_value();

  VectorXd estimated_tree_state;
  impl_->ImplDiscreteUpdate(
        q0, tree_q_measurement, point_cloud, depth_image,
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

#include "ManipulationTracker.hpp"
#include "costs/RobotStateCost.hpp"
#include "costs/JointStateCost.hpp"
#include "costs/KinectFrameCost.hpp"
#include "costs/DynamicsCost.hpp"
#include "costs/NonpenetratingObjectCost.hpp"
#include "yaml-cpp/yaml.h"
#include "common/common.hpp"

/**
 * HACK
 * Handle loading ManipluationTracker and associated costs, and provide access
 * to loaded costs.
 */
class ManipulationTrackerLoader {
 public:
  template <typename T>
  using CostList = std::vector<std::shared_ptr<T>>;

  CostList<RobotStateCost> robot_state_costs_;
  CostList<JointStateCost> joint_state_costs_;
  CostList<DynamicsCost> dynamics_costs_;
  CostList<KinectFrameCost> kinect_frame_costs_;
  CostList<NonpenetratingObjectCost> nonpentrating_costs_;
  std::shared_ptr<ManipulationTracker> estimator_;

  ManipulationTrackerLoader(const YAML::Node& config,
      const std::string& drc_path,
      std::shared_ptr<lcm::LCM> lcm, const CameraInfo* camera_info);
};

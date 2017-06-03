#include "ManipulationTrackerLoader.hpp"

using namespace std;
using namespace Eigen;

template<typename T>
shared_ptr<T> infer_shared(T* value) {
  return shared_ptr<T>(value);
}

ManipulationTrackerLoader::ManipulationTrackerLoader(const YAML::Node& config,
                                     const std::string& drc_path,
                                     std::shared_ptr<lcm::LCM> lcm)
{
  VectorXd x0_robot;
  std::shared_ptr<const RigidBodyTreed> robot = setupRobotFromConfig(config, x0_robot, string(drc_path), true, false);

  // initialize tracker itself
  ManipulationTracker estimator(robot, x0_robot, lcm, config, true);

  // and register all of the costs that we know how to handle
  if (config["costs"]){
    for (auto iter = config["costs"].begin(); iter != config["costs"].end(); iter++){
      std::string cost_type = (*iter)["type"].as<string>();
      if (cost_type == "RobotStateCost"){
        auto cost = infer_shared(new RobotStateCost(robot, lcm, *iter));
        estimator.addCost(cost);
        robot_state_costs_.push_back(cost);
      } else if (cost_type == "KinectFrameCost") {
        // demands a modifiable copy of the robot to do collision calls
        auto cost = infer_shared(new KinectFrameCost(setupRobotFromConfig(config, x0_robot, string(drc_path), true, false), lcm, *iter));
        estimator.addCost(cost);
        kinect_frame_costs_.push_back(cost);
      } else if (cost_type == "DynamicsCost") {
        auto cost = infer_shared(new DynamicsCost(robot, lcm, *iter));
        estimator.addCost(cost);
        dynamics_costs_.push_back(cost);
      } else if (cost_type == "JointStateCost") {
        auto cost = infer_shared(new JointStateCost(robot, lcm, *iter));
        estimator.addCost(cost);
        joint_state_costs_.push_back(cost);
      } else if (cost_type == "NonpenetratingObjectCost") {
        // demands a modifiable copy of the robot to do collision calls
        // also requires a list of all other robots in the scene, minus those excluded in "penetrable"
        std::string object_str = (*iter)["nonpenetrating_robot"].as<std::string>();

        std::vector<int> collider_index_correspondences;
        std::vector<std::string> non_colliders = (*iter)["penetrable_robots"].as<std::vector<std::string>>(); // confirmed: this clones rep, can mutate new var without mutating parsed YAML
        non_colliders.push_back(object_str);

        std::vector<int> object_index_correspondences;
        std::vector<std::string> object_vec;
        object_vec.push_back(object_str);

        auto cost = infer_shared(new NonpenetratingObjectCost(
          setupRobotFromConfigSubset(config, x0_robot, string(drc_path), true, false, true, non_colliders, collider_index_correspondences), collider_index_correspondences,
          setupRobotFromConfigSubset(config, x0_robot, string(drc_path), true, false, false, object_vec, object_index_correspondences), object_index_correspondences,
          lcm, *iter));
        estimator.addCost(cost);
        nonpentrating_costs_.push_back(cost);
      } else {
        cout << "Got cost type " << cost_type << " but I don't know what to do with it!" << endl;
      }
    }
  }
}

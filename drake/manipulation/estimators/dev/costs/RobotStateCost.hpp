#ifndef ROBOT_STATE_COST_H
#define ROBOT_STATE_COST_H

#include <stdexcept>
#include <iostream>
#include "ManipulationTrackerCost.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <mutex>
#include "yaml-cpp/yaml.h"

#include "lcmtypes/bot_core/robot_state_t.hpp"

#include "drake/manipulation/estimators/dev/vector_slice.h"

// TODO(eric.cousineau): Merge this with JointStateCost.

using drake::manipulation::VectorSlice;

class RobotStateCost : public ManipulationTrackerCost {
public:
  RobotStateCost(std::shared_ptr<const RigidBodyTreed> robot_, std::shared_ptr<lcm::LCM> lcm_, YAML::Node config);
  ~RobotStateCost() {};

  bool constructCost(ManipulationTracker * tracker, const Eigen::VectorXd x_old, Eigen::MatrixXd& Q, Eigen::VectorXd& f, double& K);

  void readTreeState(const Eigen::VectorXd& q,
                     const VectorSlice& slice);
private:
  std::string state_channelname = "EST_ROBOT_STATE";
  double joint_known_encoder_var = INFINITY;
  double joint_known_floating_base_var = INFINITY;
  double timeout_time = 0.5;
  bool verbose = false;
  bool transcribe_floating_base_vars = false;

  std::shared_ptr<lcm::LCM> lcm;
  lcm::Subscription * state_sub;
  std::shared_ptr<const RigidBodyTreed> robot;
  int nq;

  Eigen::VectorXd x_robot_measured;
  std::vector<bool> x_robot_measured_known;
  std::mutex x_robot_measured_mutex;

  double lastReceivedTime;
};

#endif

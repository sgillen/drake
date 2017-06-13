#undef NDEBUG
#include <assert.h> 
#include <fstream>
#include "RobotStateCost.hpp"
#include "drake/util/convexHull.h"
#include <cmath>
#include "common/common.hpp"

using namespace std;
using namespace Eigen;

RobotStateCost::RobotStateCost(std::shared_ptr<const RigidBodyTreed> robot_, std::shared_ptr<lcm::LCM> lcm_, YAML::Node config) :
    lcm(lcm_),
    robot(robot_),
    nq(robot->get_num_positions())
{
  if (config["state_channelname"])
    state_channelname = config["state_channelname"].as<string>();
  if (config["joint_known_encoder_var"])
    joint_known_encoder_var = config["joint_known_encoder_var"].as<double>();
  if (config["joint_known_floating_base_var"])
    joint_known_floating_base_var = config["joint_known_floating_base_var"].as<double>();
  if (config["timeout_time"])
    timeout_time = config["timeout_time"].as<double>();
  if (config["verbose"])
    verbose = config["verbose"].as<bool>();
  if (config["transcribe_floating_base_vars"])
    transcribe_floating_base_vars = config["transcribe_floating_base_vars"].as<bool>();

  lastReceivedTime = getUnixTime() - timeout_time*2.;
  
  x_robot_measured.resize(nq);
  x_robot_measured_known.resize(nq);
}

/***********************************************
            KNOWN POSITION HINTS
*********************************************/
bool RobotStateCost::constructCost(ManipulationTracker * tracker, const Eigen::VectorXd x_old, Eigen::MatrixXd& Q, Eigen::VectorXd& f, double& K)
{
  double now = getUnixTime();
  if (now - lastReceivedTime > timeout_time){
    if (verbose)
      printf("RobotStateCost: constructed but timed out\n");
    return false;
  }
  else {
    now = getUnixTime();

    double JOINT_KNOWN_ENCODER_WEIGHT = std::isinf(joint_known_encoder_var) ? 0.0 : 1. / (2. * joint_known_encoder_var * joint_known_encoder_var);
    double JOINT_KNOWN_FLOATING_BASE_WEIGHT = std::isinf(joint_known_floating_base_var) ? 0.0 : 1. / (2. * joint_known_floating_base_var * joint_known_floating_base_var);

    // copy over last known info
    x_robot_measured_mutex.lock();
    VectorXd q_measured = x_robot_measured.block(0,0,nq,1);
    std::vector<bool> x_robot_measured_known_copy = x_robot_measured_known;
    x_robot_measured_mutex.unlock();

    // min (x - x')^2
    // i.e. min x^2 - 2xx' + x'^2

    for (int i=0; i<6; i++){
      if (x_robot_measured_known_copy[i]){
        Q(i, i) += JOINT_KNOWN_FLOATING_BASE_WEIGHT*1.0;
        f(i) -= JOINT_KNOWN_FLOATING_BASE_WEIGHT*q_measured(i);
        K += JOINT_KNOWN_FLOATING_BASE_WEIGHT*q_measured(i)*q_measured(i);
      }
    }
    for (int i=6; i<nq; i++){
      if (x_robot_measured_known_copy[i]){
        Q(i, i) += JOINT_KNOWN_ENCODER_WEIGHT*1.0;
        f(i) -= JOINT_KNOWN_ENCODER_WEIGHT*q_measured(i);
        K += JOINT_KNOWN_ENCODER_WEIGHT*q_measured(i)*q_measured(i);
      }
    }
    if (verbose)
      printf("Spent %f in robot reported state constraints, channel %s\n", getUnixTime() - now, state_channelname.c_str());
    return true;
  }
}

void RobotStateCost::readTreeState(const VectorXd& q, const VectorSlice& slice)
{
  lastReceivedTime = getUnixTime();
  slice.WriteToSuperset(q, x_robot_measured);
  for (int i = 0; i < (int)x_robot_measured_known.size(); ++i) {
    x_robot_measured_known[i] = false;
  }
  for (int i : slice.indices()) {
    x_robot_measured_known[i] = true;
  }
}

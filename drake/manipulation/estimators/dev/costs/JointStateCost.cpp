#undef NDEBUG
#include <assert.h> 
#include <fstream>
#include "JointStateCost.hpp"
#include "drake/util/convexHull.h"
#include <cmath>
#include "common/common.hpp"

using namespace std;
using namespace Eigen;

JointStateCost::JointStateCost(std::shared_ptr<const RigidBodyTreed> robot_, std::shared_ptr<lcm::LCM> lcm_, YAML::Node config) :
    lcm(lcm_),
    robot(robot_),
    nq(robot->get_num_positions())
{
  if (config["state_channelname"])
    state_channelname = config["state_channelname"].as<string>();
  if (config["joint_reported_var"])
    joint_reported_var = config["joint_reported_var"].as<double>();
  if (config["timeout_time"])
    timeout_time = config["timeout_time"].as<double>();
  if (config["verbose"])
    verbose = config["verbose"].as<bool>();
  if (config["listen_joints"])
    for (auto iter=config["listen_joints"].begin(); iter!=config["listen_joints"].end(); iter++)
      listen_joints.push_back((*iter).as<string>());

//  if (state_channelname.size() > 0){
//    state_sub = lcm->subscribe(state_channelname, &JointStateCost::handleJointStateMsg, this);
//    state_sub->setQueueCapacity(1);
//  } else {
//    printf("JointStateCost was not handed a state_channelname, so I'm not doing anything!\n");
//  }

  lastReceivedTime = getUnixTime() - timeout_time*2.;
  
  q_robot_measured.resize(nq);
  q_robot_measured_known.resize(nq);
}

/***********************************************
            KNOWN POSITION HINTS
*********************************************/
bool JointStateCost::constructCost(ManipulationTracker * tracker, const Eigen::VectorXd x_old, Eigen::MatrixXd& Q, Eigen::VectorXd& f, double& K)
{
  double now = getUnixTime();
  if (now - lastReceivedTime > timeout_time){
    if (verbose)
      printf("JointStateCost: constructed but timed out\n");
    return false;
  }
  else {
    now = getUnixTime();

    double JOINT_REPORTED_WEIGHT = std::isinf(joint_reported_var) ? 0.0 : 1. / (2. * joint_reported_var * joint_reported_var);

    // copy over last known info
    x_robot_measured_mutex.lock();
    VectorXd q_measured = q_robot_measured.block(0,0,nq,1);
    std::vector<bool> x_robot_measured_known_copy = q_robot_measured_known;
    x_robot_measured_mutex.unlock();

    // min (x - x')^2
    // i.e. min x^2 - 2xx' + x'^2

    for (int i=0; i<nq; i++){
      if (x_robot_measured_known_copy[i]){
        Q(i, i) += JOINT_REPORTED_WEIGHT*1.0;
        f(i) -= JOINT_REPORTED_WEIGHT*q_measured(i);
        K += JOINT_REPORTED_WEIGHT*q_measured(i)*q_measured(i);
      }
    }
    if (verbose)
      printf("Spent %f in joint reported state constraints, channel %s.\n", getUnixTime() - now, state_channelname.c_str());
    return true;
  }
}

void JointStateCost::readTreeState(const Eigen::VectorXd& q,
                                   const VectorSlice<double>& slice) {
  slice.WriteToSuperset(q, q_robot_measured);
  for (int i = 0; i < (int)q_robot_measured_known.size(); ++i) {
    q_robot_measured_known[i] = false;
  }
  for (int i : slice.indices()) {
    q_robot_measured_known[i] = true;
  }
}

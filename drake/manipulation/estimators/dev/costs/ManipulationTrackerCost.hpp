#ifndef MANIPULATION_TRACKER_COST_H
#define MANIPULATION_TRACKER_COST_H

#include <Eigen/Dense>
#include "ManipulationTracker.hpp"

// forward declaration
class ManipulationTracker;

class ManipulationTrackerCost {
  public:
    // handed references to Q, f matrices for the cost function to construct
    // returns a bool, which indicates whether this cost should be used this cycle
    virtual bool constructCost(ManipulationTracker * tracker, const Eigen::VectorXd x_old, Eigen::MatrixXd& Q, Eigen::VectorXd& f, double& K) { return false; }
    
    // handed reference reference to a vector of this cost's personal decision variables
    // (NO robot state) and a square matrix of that width.
    // should populate the vector with updates to the state variables,
    // and populate the matrix with a new covariance estimate (to be combined with other costs)
    // returns bool, returning whether these updates should be used this cycle
    virtual bool constructPredictionMatrices(ManipulationTracker * tracker, const Eigen::VectorXd x_old, Eigen::VectorXd& x, Eigen::MatrixXd& P) { return false; }

    int getNumExtraVars(void) { return num_extra_vars_; }
    Eigen::VectorXd getExtraVarsX0(void) { return extra_vars_x0_; }

  protected:
  	int num_extra_vars_ = 0; // extra decision variables needed by this cost. must be set in constructor
    Eigen::VectorXd extra_vars_x0_;
};

#endif
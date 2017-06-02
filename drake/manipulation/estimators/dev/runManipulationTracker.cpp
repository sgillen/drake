
#include "ManipulationTracker.hpp"
#include "costs/RobotStateCost.hpp"
#include "costs/JointStateCost.hpp"
#include "costs/KinectFrameCost.hpp"
#include "costs/DynamicsCost.hpp"
// #include "costs/GelsightCost.hpp"
// #include "costs/AttachedApriltagCost.hpp"
// #include "costs/OptotrakMarkerCost.hpp"
#include "costs/NonpenetratingObjectCost.hpp"
#include "yaml-cpp/yaml.h"
#include "common/common.hpp"
#include "ManipulationTrackerLoader.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  const char* drc_path = std::getenv("DRC_BASE");
  if (!drc_path) {
    throw std::runtime_error("environment variable DRC_BASE is not set");
  }

  if (argc != 2){
    printf("Use: runManipulationTrackerIRB140 <path to yaml config file>\n");
    return 0;
  }


  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM());
  if (!lcm->good()) {
    throw std::runtime_error("LCM is not good");
  }

  string configFile(argv[1]);
  YAML::Node config = YAML::LoadFile(configFile);

  ManipulationTrackerLoader loader(config, drc_path, lcm);
  ManipulationTracker& estimator = *loader.estimator_;

  std::cout << "Manipulation Tracker Listening" << std::endl;

  double last_update_time = getUnixTime();
  double timestep = 0.01;
  if (config["timestep"])
    timestep = config["timestep"].as<double>();

  while(1){
    for (int i=0; i < 100; i++)
      lcm->handleTimeout(0);

    double dt = getUnixTime() - last_update_time;
    if (dt > timestep){
      last_update_time = getUnixTime();
      estimator.update();
      estimator.publish();
    } else {
      usleep(1000);
    }
  }

  return 0;
}

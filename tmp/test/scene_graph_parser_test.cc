#include "drake/tmp/scene_graph_parser.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"

namespace drake {
namespace geometry {

GTEST_TEST(SceneGraphParser, TestLifetime) {
  SceneGraph<double> scene_graph;
  SceneGraphParser parser(&scene_graph);
  parser.AddModelFromFile(
      FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.sdf"),
      "acrobot1");
  parser.AddModelFromFile(
      FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.sdf"),
      "acrobot2");
  auto id = parser.Finalize();
  unused(id);

  // Dunno about ordering.
  lcm::DrakeLcm lcm;
  // See if we get segfaults...
  DispatchLoadMessage(scene_graph, &lcm);
}

}
}

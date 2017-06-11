#include "dart.h"

int main() {
  // Keep name mangling outside of Dart estimation???
  DartScene scene(tree, instance_ids);

  DartEstimator estimator(scene, estimated_joints);
  auto* joint =
    estimator.AddObjective<DartJointObjective>(names, ic, weights);
  auto* depth =
    estimator.AddObjective<DartDepthImageIcpObjective>(...);
  auto* nonpen =
    estimator.AddObjective<DartNonpenetrationObjective>(...);

  // Updates...
  estimator.NewObservationFrame(t);
  joint->ObserveKinematicState(t, q, v);
  depth->ObserveImage(t, depth_image, frame);
  nonpen->Observe(t);

  // Update estimator
  tie(q, v) = estimator.Update();
}

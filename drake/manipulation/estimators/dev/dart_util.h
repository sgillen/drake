#include <memory>
#include <vector>
#include <map>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/common/symbolic_variables.h"
#include "drake/solvers/decision_variable.h"

#include "drake/manipulation/estimators/dev/vector_slice.h"

namespace drake {
namespace manipulation {

using namespace std;
using namespace Eigen;

typedef RigidBodyFrame<double> RigidBodyFramed;
typedef KinematicsCache<double> KinematicsCached;
typedef Matrix6X<double> Matrix6Xd;

typedef shared_ptr<const RigidBodyTreed> TreePtr;
typedef vector<int> Indices;
typedef map<string, int> InstanceIdMap;
typedef map<string, double> JointWeights;
typedef solvers::VectorXDecisionVariable OptVars;

/**
 * Convenience to infer type.
 */
template <typename T>
std::unique_ptr<T> CreateUnique(T* obj) {
  return std::unique_ptr<T>(obj);
}

/**
 * Compute Cartesian jacobian of a point in the world, but attached to a body.
 * This will transform the point to the body frame of interest, and then compute
 * the relevant Jacobian.
 */
inline MatrixXd ComputeJacobianWithWorldPoints(
  const RigidBodyTreed& tree, const KinematicsCached& cache,
  const Vector3d& pt_W, const RigidBodyFramed* frame_A,
  bool in_terms_of_qdot) {
  // TODO(eric.cousineau): Check if this should require a body, not a frame,
  // to ensure additional constants don't sneak in.
  int frame_index = frame_A ? frame_A->get_frame_index() : 0;
  if (frame_index != 0) {
    Vector3d pt_A = tree.transformPoints(cache, pt_W, 0, frame_index);
    return tree.transformPointsJacobian(
      cache, pt_A, frame_index, 0, in_terms_of_qdot);
  } else {
    int nvar = in_terms_of_qdot ?
        tree.get_num_positions() : tree.get_num_velocities();
    return Matrix6Xd::Zero(6, nvar);
  }
}

/**
 * Compute the error in Cartesian position between two points in the world frame
 * `W`, and yield the affine (constant) and linear Jacobian of the error w.r.t.
 * the RigidBodyTree variables, for use in things such as a L2 error norm cost:
 *   e_lin(de) = e + Je*de
 * Frame A may be null, in which case it is assumed to be the world.
 *
 * This will perform the transformation in terms of q_dot for use in
 * optimization.
 */
inline void ComputeIcpPointToPointError(
  const RigidBodyTreed& tree, const KinematicsCached& cache,
  const RigidBodyFramed* frame_A, const Vector3d& ptA_W,
  const RigidBodyFramed* frame_B, const Vector3d& ptB_W,
  Vector3d* perror_W, MatrixXd* pJerror_W) {
  DRAKE_DEMAND(perror_W);
  DRAKE_DEMAND(pJerror_W);
  DRAKE_DEMAND(frame_B);

  auto& error_W = *perror_W;
  auto& Jerror_W = *pJerror_W;

  error_W = ptA_W - ptB_W;
  if (frame_A) {
    Jerror_W = ComputeJacobianWithWorldPoints(
        tree, cache, ptA_W, frame_A, true).topRows<3>();
  } else {
    Jerror_W.setZero();
  }
  Jerror_W -= ComputeJacobianWithWorldPoints(
        tree, cache, ptB_W, frame_B, true).topRows<3>();
}

// TODO(eric.cousineau): Merge functionality into WorldSimTreeBuilder.
// TODO(eric.cousineau): Find better location for this.
typedef std::map<int, std::string> ReverseIdMap;

void PrintJointNameHierarchy(const RigidBodyTreed* tree);

std::vector<std::string> GetHierarchicalPositionNameList(const RigidBodyTreed& tree,
    const ReverseIdMap& instance_name_map, bool add_velocity = false);

inline void PrintValidPoints(const Eigen::Matrix3Xd& points,
                             const std::string& note) {
  int num_non_nan = 0;
  for (int i = 0; i < points.cols(); ++i) {
    if (!std::isnan(points(0, i))) {
      num_non_nan += 1;
    }
  }
  std::cout << fmt::format("Valid points: {} - {}\n", num_non_nan, note);
}

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

}  // manipulation
}  // drake

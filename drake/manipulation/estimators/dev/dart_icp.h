#pragma once

#include "drake/common/nice_type_name.h"
#include "drake/manipulation/estimators/dev/dart_util.h"
#include "drake/solvers/cost.h"

namespace drake {
namespace manipulation {

typedef int BodyIndex;
typedef int FrameIndex;

struct Interval {
  double min{};
  double max{};
  inline bool IsInside(double i) const {
    return i >= min && i <= max;
  }
  inline double width() const {
    return max - min;
  }
};

struct Bounds {
  Interval x;
  Interval y;
  Interval z;
  inline bool IsInside(double xi, double yi, double zi) const {
    return x.IsInside(xi) && y.IsInside(yi) && z.IsInside(zi);
  }
};

struct IcpPointCorrespondence {
  // TODO(eric.cousineau): Consider storing normals, different frames, etc.
  /** @brief Measured feature index (point). */
  int meas_index{};
  /** @brief Measured point. */
  Vector3d meas_point;
  /** @brief Model feature index (body). */
  int model_index{};
  /** @brief Model point, same frame as measured point. */
  Vector3d model_point;
  /** @brief Distance between the points. */
  double distance{-1};
};

typedef map<BodyIndex, vector<IcpPointCorrespondence>> IcpSceneCorrespondences;

struct BodyCorrespondenceInfluence {
  /**
   * @brief Will a correspondence with this body affect the camera
   * positions?
   */
  bool will_affect_camera{};
  /**
   * @brief Will a correspondence with this body affect the body's kinematics?
   */
  bool will_affect_body{};
  /**
   * @brief Check if a correspondence with this body will influence the
   * Jacobian at all.
   */
  bool is_influential() const {
    return will_affect_camera || will_affect_body;
  }
};
typedef vector<BodyCorrespondenceInfluence> Influences;

/**
 * Accumulate measured points in the camera `C` frame, body / model /
 * simulated points in the `Bi` frame, and store both in the world frame,
 * `W`, if available.
 */
struct IcpScene {
  IcpScene(const RigidBodyTreed& tree,
           const KinematicsCached& cache,
           const FrameIndex frame_W,
           const FrameIndex frame_C)
    : tree(tree),
      cache(cache),
      frame_W(frame_W),
      frame_C(frame_C) {}
  const RigidBodyTreed& tree;
  // TODO(eric.cousineau): Remove cache, as this contains state. Rename
  // something similar to IcpSceneState.
  const KinematicsCached& cache;
  const FrameIndex frame_W;
  const FrameIndex frame_C;
};

//struct IcpSceneState {
//  IcpSceneState(const IcpScene& scene,
//                KinematicsCached& cache)
//      : scene(scene),
//        cache(cache) {}
//  const IcpScene& scene;
//  KinematicsCached& cache;
//};

/**
 * A group of points to be rendered in a linearized ICP cost.
 */
struct IcpBodyPoints {
  IcpBodyPoints(FrameIndex frame_Bi, int num_max)
    : frame_Bi(frame_Bi) {
    meas_pts_W.resize(3, num_max);
    body_pts_W.resize(3, num_max);
  }
  void Add(const Vector3d& meas_W, const Vector3d& body_W) {
    int i = num_actual++;
    meas_pts_W.col(i) = meas_W;
    body_pts_W.col(i) = body_W;
  }
  void Finalize() {
    meas_pts_W.conservativeResize(NoChange, num_actual);
    body_pts_W.conservativeResize(NoChange, num_actual);
  }
  void ComputeError(
      const IcpScene& scene, Matrix3Xd *es_W, MatrixXd *J_es_W) const {
    // Compute measured and body (model) points in their respective frames for
    // Jacobian computation.
    DRAKE_DEMAND(es_W != nullptr);
    // Compute error
    *es_W = meas_pts_W - body_pts_W;

    // Get point jacobian w.r.t. camera frame, as that is the only influence
    // on the measured point cloud.
    // TODO(eric.cousineau): If camera is immovable w.r.t. formulation, do not
    // update. Consider passing in body influence information.
    if (J_es_W) {
      Matrix3Xd meas_pts_C = scene.tree.transformPoints(
              scene.cache, meas_pts_W, scene.frame_W, scene.frame_C);
      MatrixXd J_meas_pts_W =
          scene.tree.transformPointsJacobian(
              scene.cache, meas_pts_C, scene.frame_C, scene.frame_W, false);
      // TODO(eric.cousineau): If body is immovable w.r.t. formulation, do not
      // update.
      Matrix3Xd body_pts_Bi = scene.tree.transformPoints(
          scene.cache, body_pts_W, scene.frame_W, frame_Bi);
      MatrixXd J_body_pts_W =
          scene.tree.transformPointsJacobian(
              scene.cache, body_pts_Bi, frame_Bi, scene.frame_W, false);
      // Compute Jacobian of error.
      *J_es_W = J_meas_pts_W - J_body_pts_W;
    }
  }
 private:
  const FrameIndex frame_Bi{-1};
  Matrix3Xd meas_pts_W;
  Matrix3Xd body_pts_W;
  int num_actual{0};
};

/**
 * Will a correspondence with a given body influence the optimization
 * formulation (camera or body position)? Will it mathematically affect
 * anything, or should computations relevant to this be skipped?
 * @param scene
 * @param ComputeCorrespondences
 */
BodyCorrespondenceInfluence IsBodyCorrespondenceInfluential(
    const IcpScene& scene,
    BodyIndex body_index,
    const VectorSlice *q_slice) {
  BodyCorrespondenceInfluence out;
  out.will_affect_camera = (scene.frame_C != scene.frame_W);
  if (q_slice) {
    // Check if this body's kinematic chain will be affected by the slice.
    vector<int> q_indices;
    GetDofPath(scene.tree, body_index, &q_indices);
    // If there is any intersection, then this will influence things.
    out.will_affect_body =
        HasIntersection(q_indices, q_slice->indices());
  } else {
    out.will_affect_body = true;
  }
  return out;
}

void ComputeBodyCorrespondenceInfluences(
    const IcpScene& scene,
    const VectorSlice* q_slice,
    Influences* influences) {
  int num_bodies = scene.tree.get_num_bodies();
  influences->resize(num_bodies);
  for (int i = 0; i < num_bodies; ++i) {
    (*influences)[i] = IsBodyCorrespondenceInfluential(scene, i, q_slice);
  }
}

/**
 * This function depends on `q`.
 * @param scene
 * @param meas_pts_W
 * @param pcorrespondence
 */
void ComputeCorrespondences(const IcpScene& scene,
                            const Matrix3Xd& meas_pts_W,
                            const Influences& influences,
                            IcpSceneCorrespondences* pcorrespondence) {
  DRAKE_DEMAND(pcorrespondence != nullptr);
  int num_points = meas_pts_W.cols();
  VectorXd distances(num_points);
  Matrix3Xd body_normals_W(3, num_points);  // world frame.
  Matrix3Xd body_pts_W(3, num_points);  // body point, world frame.
  Matrix3Xd body_pts_Bi(3, num_points);  // body point, body frame.
  vector<BodyIndex> body_indices(num_points);

  const bool use_margins = false;
  // TOOD(eric.cousineau): Figure out better access.
  RigidBodyTreed& mutable_tree =
      const_cast<RigidBodyTreed&>(scene.tree);
  mutable_tree.collisionDetectFromPoints(
      scene.cache, meas_pts_W,
      distances, body_normals_W,
      body_pts_W, body_pts_Bi,
      body_indices, use_margins);

  // Cache whether or not there is influence. This should be effectively used
  // to filter out points from the set.
  int num_bodies = scene.tree.get_num_bodies();
  vector<bool> is_body_influential(num_bodies);
  for (int i = 0; i < num_bodies; ++i) {
    is_body_influential[i] = influences[i].is_influential();
  }

  // Bin each correspondence to the given body.
  for (int i = 0; i < num_points; ++i) {
    BodyIndex body_index = body_indices[i];
    if (body_index != -1 && is_body_influential[body_index]) {
      vector<IcpPointCorrespondence>& body_correspondences =
          (*pcorrespondence)[body_index];
      IcpPointCorrespondence pc{
          i, meas_pts_W.col(i), body_index, body_pts_W.col(i), distances[i]};
      body_correspondences.push_back(pc);
    }
  }
}

typedef std::function<void(const IcpBodyPoints&)> CostAggregatorFunctor;

void AggregateCost(const IcpScene& scene,
                   const IcpSceneCorrespondences& correspondence,
                   const CostAggregatorFunctor& aggregate) {
  for (const auto& pair : correspondence) {
    const BodyIndex body_index = pair.first;
    const vector<IcpPointCorrespondence>& body_correspondence = pair.second;
    IcpBodyPoints body_pts(body_index, body_correspondence.size());
    for (const auto& pc_W : body_correspondence) {
      body_pts.Add(pc_W.meas_point, pc_W.model_point);
    }
    body_pts.Finalize();
    aggregate(body_pts);
  }
}

/**
 * Aggregate normalized cost of ICP points.
 */
// TODO(eric.cousineau): Consider merging with QP cost aggregator.
class IcpCostAggregator {
 public:
  IcpCostAggregator(const IcpScene& scene)
    : scene_(&scene), cost_(0) {}

  void operator()(const IcpBodyPoints& body_pts) {
    body_pts.ComputeError(*scene_, &es_, &Jes_);
    // Get error squared.
    cost_ += (es_.transpose() * es_).sum();
    Jcost_.resize(1, Jes_.cols());
    for (int i = 0; i < es_.cols(); ++i) {
      auto&& e = es_.col(i);
      auto&& Je = Jes_.middleRows(3 * i, 3);
      Jcost_ += 2 * e.transpose() * Je;
    }
  }
  double cost() const {
    return cost_;
  }
  const MatrixXd& cost_jacobian() const {
    return Jcost_;
  }
 private:
  const IcpScene* scene_;
  Matrix3Xd es_;
  MatrixXd Jes_;
  MatrixXd Jcost_;
  double cost_{};
};

/**
 * Accumulate quadratic point-to-point errors to be rendered into a
 * QuadraticCost.
 * This will accumulate in all joint coordinates. Extracting indices from there
 * is delegated to another component.
 *
 * Linearized: | e + J*q |^2
 */
class IcpLinearizedCostAggregator {
 public:
  IcpLinearizedCostAggregator(const IcpScene& scene)
      : scene_(&scene) {
    const int nvar = scene.tree.get_num_positions();
    Q_.resize(nvar, nvar);
    b_.resize(nvar);
    Clear();
  }

  void Clear() {
    Q_.setZero();
    b_.setZero();
    c_ = 0;
  }

  double cost() const { return c_; }

  void operator()(const IcpBodyPoints& body_pts_group) {
    const VectorXd& q0 = scene_->cache.getQ();
    body_pts_group.ComputeError(*scene_, &es_, &Jes_);
    int num_points = es_.cols();
    for (int i = 0; i < num_points; ++i) {
      auto&& e = es_.col(i);
      auto&& Je = Jes_.middleRows(3 * i, 3);
      // Norm of linearization: |e + J*(q - q0)|^2
      Vector3d k = e - Je * q0;
      Q_ += 2 * Je.transpose() * Je;
      b_ += 2 * Je.transpose() * k;
      c_ += k.dot(k);
    }
  }

  void UpdateCost(const VectorSlice& slice, QuadraticCost* cost) const {
    int ncvar = slice.size();
    MatrixXd Qc(ncvar, ncvar);
    VectorXd bc(ncvar);
    slice.ReadFromSuperset(b_, bc);
    // Some inefficiency here. Could reduce before hand, but we'll just wait
    // for summation to complete.
    slice.ReadFromSupersetMatrix(Q_, Qc);
    cost->UpdateCoefficients(Qc, bc, c_);
  }

  void PrintDebug(bool all_points) const {
    int num_points = es_.cols();
    cout << "n: " << num_points << endl;
    for (int i = 0; i < num_points; ++i) {
      auto&& e = es_.col(i);
      cout << " - i: " << e.transpose() << endl;
    }
  }

 private:
  const IcpScene* scene_;
  MatrixXd Q_;
  VectorXd b_;
  double c_;
  Matrix3Xd es_;
  MatrixXd Jes_;
};

}  // namespace manipulation
}  // namespace drake

#pragma once

#include "drake/manipulation/estimators/dev/dart.h"
#include "drake/manipulation/estimators/dev/dart_objectives.h"

#include "drake/systems/sensors/image.h"

namespace drake {
namespace manipulation {

// Copied from RgbdCamera.
const int kImageWidth = 640;  // In pixels
const int kImageHeight = 480;  // In pixels

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

/**
 * Accumulate quadratic point-to-point errors to be rendered into a
 * QuadraticCost.
 * This will accumulate in all joint coordinates. Extracting indices from there
 * is delegated to another component.
 *
 * Normalize | e + J*q |^2
 */
struct IcpLinearizedNormAccumulator {
 public:
  IcpLinearizedNormAccumulator(int nvar) {
    Q_.resize(nvar, nvar);
    b_.resize(nvar);
    Clear();
  }
  void Clear() {
    Q_.setZero();
    b_.setZero();
    c_ = 0;
  }
  void AddTerms(double weight, const Matrix3Xd& es, const MatrixXd& Jes) {
    int num_points = es.cols();
    for (int i = 0; i < num_points; ++i) {
      auto&& e = es.col(i);
      auto&& Je = Jes.middleRows(3 * i, 3);
      Q_ += weight * 2 * Je.transpose() * Je;
      b_ += weight * 2 * Je.transpose() * e;
      c_ = weight * e.dot(e);
    }
  }
  const MatrixXd& Q() const { return Q_; }
  const VectorXd& b() const { return b_; }
  double c() const { return c_; }
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
 private:
  MatrixXd Q_;
  VectorXd b_;
  double c_;
};

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
  const KinematicsCached& cache;
  const FrameIndex frame_W;
  const FrameIndex frame_C;
};

/**
 * A group of points to be rendered in a linearized ICP cost.
 */
struct IcpPointGroup {
  IcpPointGroup(FrameIndex frame_Bi, int num_max)
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
    auto trim_pts = [this](auto&& X) {
      X.conservativeResize(NoChange, num_actual);
    };
    trim_pts(meas_pts_W);
    trim_pts(body_pts_W);
  }
  void UpdateError(
      const IcpScene& scene, Matrix3Xd* es_W, MatrixXd* J_es_W) const {
    // Compute measured and body (model) points in their respective frames for
    // Jacobian computation.
    Matrix3Xd meas_pts_C;
    Matrix3Xd body_pts_Bi;
    scene.tree.transformPoints(
        scene.cache, meas_pts_W, scene.frame_W, scene.frame_C);
    scene.tree.transformPoints(
            scene.cache, body_pts_W, scene.frame_W, frame_Bi);

    // Get point jacobian w.r.t. camera frame, as that is the only influence
    // on the measured point cloud.
    // TODO(eric.cousineau): If camera is immovable w.r.t. formulation, do not
    // update. Consider passing in body influence information.
    MatrixXd J_meas_pts_W =
        scene.tree.transformPointsJacobian(
            scene.cache, meas_pts_C, scene.frame_C, scene.frame_W, false);
    // TODO(eric.cousineau): If body is immovable w.r.t. formulation, do not
    // update.
    MatrixXd J_body_pts_W =
        scene.tree.transformPointsJacobian(
            scene.cache, body_pts_Bi, frame_Bi, scene.frame_W, false);
    // Compute error and its Jacobian.
    *es_W = meas_pts_W - body_pts_W;
    *J_es_W = J_meas_pts_W - J_body_pts_W;
  }
 private:
  const FrameIndex frame_Bi{-1};
  Matrix3Xd meas_pts_W;
  Matrix3Xd body_pts_W;
  int num_actual{0};
};

/**
 * ICP-based lineraized QP-based cost objective for both S_mod (positive
 * returns) and S_obs (negative returns).
 */
class DartDepthImageIcpObjective : public DartObjective {
 public:
  struct Param {
    // TODO(eric.cousineau): Incorporate camera parameters.
    struct Camera {
      double fov_y{M_PI / 4};
      bool show_window{false};
      shared_ptr<RigidBodyFramed> frame;
    };
    Camera camera;
    // Sub-sampling.
    int image_downsample_factor{5};
    Bounds point_cloud_bounds {
      .x = {-1, 1},
      .y = {-1, 1},
      .z = {-1, 1},
    };
    // Weights.
    struct Icp {
      double variance{0.5};
      double max_distance_m{0.5};
      double min_joint_distance_m{0.05};
    };
    Icp icp;
    struct FreeSpace {
      double variance{0.005};
    };
    FreeSpace free_space;
//    struct Debug {
//      bool use_lcmgl{false};
//    };
//    Debug debug;
  };

  DartDepthImageIcpObjective(DartFormulation* formulation_, const Param& param);
  ~DartDepthImageIcpObjective();

  void Init(const KinematicsCached& cache) override;
  void ObserveImage(double t,
                    const systems::sensors::ImageDepth32F& depth_image_meas,
                    const Matrix3Xd* ppoint_cloud = nullptr);
  void UpdateFormulation(double t, const KinematicsCached& kin_cache,
                         const VectorXd &obj_prior) override;
  const OptVars& GetVars() const override { return extra_vars_; }
  const VectorXd& GetInitialValues() const override { return extra_ic_; }
 private:
  void DetermineUnaffectedBodies();

  shared_ptr<QuadraticCost> icp_cost_;
  shared_ptr<QuadraticCost> free_space_cost_;

  Param param_;
  class Impl;
  friend class Impl;
  unique_ptr<Impl> impl_;
  // Throw-away?
  VectorXd extra_ic_;
  OptVars extra_vars_;
};

}  // namespace manipulation
}  // namespace drake

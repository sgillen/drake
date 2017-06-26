#include "drake/manipulation/estimators/dev/dart_depth_image_icp_objective.h"

#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/common/scoped_timer.h"

#include <bot_lcmgl_client/lcmgl.h>
#include "drake/multibody/joints/revolute_joint.h"

using namespace drake::systems::sensors;

namespace drake {
namespace manipulation {

// TODO(eric.cousineau): Permit the cost to be plugged into a continuous, non-
// linear update (e.g., a non-linear MathematicalProgram) per Alejandro's
// suggestions.

struct Coord {
  int u{0};  // x-pixel coordinate
  int v{0};  // y-pixel coordinate
};

/**
 * Fixed-width grid slice.
 */
// TODO(eric.cousineau): Support per-index mapping.
// TODO(eric.cousineau): Use better down-sampling (anti-aliasing, etc.). Look
// into using VTK for performing this. FOLLOWUP: Nevermind. Very bad for depth
// images.
// TODO(eric.cousineau): Consider taking the most prominent (least depth) pixel
// in a region.
class GridSlice {
 public:
  GridSlice(int downsample, int super_width, int super_height)
      : downsample_(downsample),
        width_(super_width / downsample),
        height_(super_height / downsample),
        super_width_(super_width),
        super_height_(super_height) {}
  GridSlice(int width, int height)
      : GridSlice(1, width, height) {}
  GridSlice()
      : GridSlice(0, 0) {}

  int width() const { return width_; }
  int height() const { return height_; }
  int size() const { return width_ * height_; }
  int downsample() const { return downsample_; }

  int super_width() const { return super_width_; }
  int super_height() const { return super_height_; }
  int super_size() const { return super_width_ * super_height_; }

  void Check(int index) const {
    ASSERT_THROW_FMT(index >= 0 && index < size(),
                     "index = {}, size() = {}", index, size());
  }
  void Check(Coord c) const {
    ASSERT_THROW_FMT(c.u >= 0 && c.u < width() && c.v >= 0 && c.v < height(),
                     "(u, v) = ({}, {}), (width, height) = ({}, {})",
                     c.u, c.v, width(), height());
  }
  Coord IndexToCoord(int index) const {
    return {index % width(), index / width()};
  }
  int CoordToIndex(Coord c) const {
    return c.v * width() + c.u;
  }
  Coord CoordToSuper(Coord c) const {
    return {c.u * downsample_, c.v * downsample_};
  }
  /// Returns nearest point.
  Coord CoordFromSuper(Coord c_super) const {
    return {c_super.u / downsample_, c_super.v / downsample_};
  }
 private:
  int downsample_{};
  int width_{};
  int height_{};
  int super_width_{};
  int super_height_{};
};

struct Output {
  GridSlice slice;
  ImageDepth32F depth_image;
  Matrix3Xd point_cloud_C;  // `C` is the (depth) camera frame.
  void Resize(int width, int height) {
    slice = GridSlice(width, height);
    ResizeToSlice();
  }
  void ResizeToSlice() {
    depth_image.resize(slice.width(), slice.height());
    point_cloud_C.resize(NoChange, slice.size());
  }
  void DownsampleFrom(int downsample, const Output& super) {
    slice = GridSlice(downsample, super.slice.width(), super.slice.height());
    ResizeToSlice();
    Coord c;
    for (c.v = 0; c.v < slice.height(); c.v += downsample) {
      for (c.u = 0; c.u < slice.width(); c.u += downsample) {
        int index = slice.CoordToIndex(c);
        Coord c_super = slice.CoordToSuper(c);
        int index_super = super.slice.CoordToIndex(c_super);
        *depth_image.at(c.u, c.v) =
            *super.depth_image.at(c_super.u, c_super.v);
        point_cloud_C.col(index) =
            super.point_cloud_C.col(index_super);
      }
    }
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
           const int frame_W,
           const int frame_C)
    : tree(tree),
      cache(cache),
      frame_W(frame_W),
      frame_C(frame_C) {}
  const RigidBodyTreed& tree;
  const KinematicsCached& cache;
  const int frame_W;
  const int frame_C;
};

/**
 * A group of points to be rendered in a linearized ICP cost.
 */
struct IcpPointGroup {
  IcpPointGroup(int frame_Bi, int num_max)
    : frame_Bi(frame_Bi),
      num_max(num_max) {
    meas_pts_W.resize(3, num_max);
    meas_pts_W.resize(3, num_max);
    meas_pts_C.resize(3, num_max);
    body_pts_Bi.resize(3, num_max);
    body_pts_W.resize(3, num_max);
  }
  void Add(const Vector3d& meas_W, const Vector3d& meas_C,
          const Vector3d& body_W, const Vector3d& body_Bi) {
    int i = num_actual++;
    meas_pts_W.col(i) = meas_W;
    meas_pts_C.col(i) = meas_C;
    body_pts_W.col(i) = body_W;
    body_pts_Bi.col(i) = body_Bi;
  }
  void AddTerms(const IcpScene& scene,
                IcpLinearizedNormAccumulator* error_accumulator,
                double weight) {
    Trim();
    // Get point jacobian w.r.t. camera frame, as that is the only influence
    // on the measured point cloud.
    MatrixXd J_meas_pts_W =
        scene.tree.transformPointsJacobian(
            scene.cache, meas_pts_C, scene.frame_C, scene.frame_W, false);
    MatrixXd J_body_pts_W =
        scene.tree.transformPointsJacobian(
            scene.cache, body_pts_Bi, frame_Bi, scene.frame_W, false);
    // Compute errors.
    Matrix3Xd es_W = meas_pts_W - body_pts_W;
    MatrixXd J_es_W = J_meas_pts_W - J_body_pts_W;
    // Incorproate errors into L2 norm cost.
    error_accumulator->AddTerms(weight, es_W, J_es_W);
  }
 private:
  void Trim() {
    auto trim_pts = [this](auto&& X) {
      X.conservativeResize(NoChange, num_actual);
    };
    trim_pts(meas_pts_W);
    trim_pts(meas_pts_C);
    trim_pts(body_pts_W);
    trim_pts(body_pts_Bi);
  }
  const int frame_Bi{-1};
  const int num_max{};
  Matrix3Xd meas_pts_W;
  Matrix3Xd meas_pts_C;
  Matrix3Xd body_pts_Bi;
  Matrix3Xd body_pts_W;
  int num_actual{0};
};

class DartDepthImageIcpObjective::Impl {
 public:
  unique_ptr<RgbdCameraDirect> rgbd_camera_sim;
  // Measured
  Output meas_full;
  Output meas;  // Sub-sampled
  // Simulated quantities for 2D SDF
  Output sim_full;
  Output sim;
};

DartDepthImageIcpObjective::DartDepthImageIcpObjective(
    DartFormulation* formulation_,
    const DartDepthImageIcpObjective::Param& param)
    : DartObjective(formulation_),
      param_(param) {
  impl_.reset(new Impl());
  impl_->rgbd_camera_sim.reset(
        new RgbdCameraDirect(tree(),
                             *param_.camera.frame, param_.camera.fov_y,
                             param_.camera.show_window));
}

DartDepthImageIcpObjective::~DartDepthImageIcpObjective() {}

void DartDepthImageIcpObjective::Init(const KinematicsCached& cache) {
  unused(cache);
  int nq_est = q_est_vars().size();
  // Make a blank cost.
  icp_cost_ = make_shared<QuadraticCost>(
      MatrixXd::Zero(nq_est, nq_est), VectorXd::Zero(nq_est), 0);
  prog().AddCost(icp_cost_, q_est_vars());
//  prog().AddCost(free_space_cost_, q_est_vars());
}

void DartDepthImageIcpObjective::ObserveImage(
    double t, const ImageDepth32F& depth_image_meas,
    const Matrix3Xd* ppoint_cloud) {
  set_latest_observation_time(t);

  Impl& impl = *impl_;
  impl.meas_full.Resize(depth_image_meas.width(), depth_image_meas.height());
  impl.meas_full.depth_image = depth_image_meas;
  if (ppoint_cloud) {
    impl.meas_full.point_cloud_C = *ppoint_cloud;
  } else {
    // (re)Generate point cloud.
    // TODO(eric.cousineau): Only regenerate down-sampled point cloud.
    RgbdCamera::ConvertDepthImageToPointCloud(
          impl.meas_full.depth_image, impl.rgbd_camera_sim->depth_camera_info(),
          &impl.meas_full.point_cloud_C);
  }
  // Down-sample here at measurement frame.
  impl.meas.DownsampleFrom(param_.image_downsample_factor, impl.meas_full);
}

/**
 * Return distance to a unit axis starting from the origin.
 */
double GetDistanceToAxis(const Vector3d& pt, const Vector3d& n) {
  const double kEpsilon = 1e-10;
  DRAKE_ASSERT(abs(n.norm() - 1) < kEpsilon);
  return (pt - n * n.dot(pt)).norm();
}


void DartDepthImageIcpObjective::UpdateFormulation(
    double t, const KinematicsCached& kin_cache, const VectorXd& obj_prior) {
  unused(t);
  unused(obj_prior);

  // Convenience aliases.
  Impl& impl = *impl_;
  const KinematicsSlice& kin_est_slice = formulation().kinematics_est_slice();
  const GridSlice& meas_slice = impl.meas.slice;
  // HACK(eric.cousineau): Mutable for `collisionDetectFromPoints()`.
  const RigidBodyTreed& tree = this->tree();
  RigidBodyTreed& mutable_tree = const_cast<RigidBodyTreed&>(tree);

//  KinematicsState kin_state(kin_cache);
  const bool use_margins = false;

  auto IsValidPoint = [this](auto&& pt) {
    if (isnan(pt[0])) {
      return false;
    } else {
      return param_.point_cloud_bounds.IsInside(pt[0], pt[1], pt[2]);
    }
  };

  // Identify points to be used for positive returns.
  // NOTE: Emphasis that `positive` is for positive RETURNS in the sense of
  // matching the to the point clouds (attracting), not positive signs.
  int num_meas = meas_slice.size();
  vector<int> positive_meas_indices(num_meas);  // Indices in `meas.slice`.
  Matrix3Xd positive_meas_pts_W(3, num_meas);  // Point cloud points.
  int num_positive = 0;

  // Define scene configuration for world and camera.
  // TODO(eric.cousineau): Consider a configuration with multiple cameras.
  IcpScene scene(tree, kin_cache,
                 param_.camera.frame->get_frame_index(),
                 tree.world().get_body_index());

  const Matrix3Xd& meas_pts_C = impl.meas.point_cloud_C;
  Matrix3Xd meas_pts_W =
      tree.transformPoints(scene.cache, meas_pts_C,
                           scene.frame_C, scene.frame_W);
  Coord c{};
  for (c.v = 0; c.v < meas_slice.height(); c.v++) {
    for (c.u = 0; c.u < meas_slice.width(); c.u++) {
      int meas_index = meas_slice.CoordToIndex(c);
      Vector3d meas_pt_W = meas_pts_W.col(meas_index);
      if (IsValidPoint(meas_pt_W)) {
        int i = num_positive++;
        positive_meas_indices[i] = meas_index;
        positive_meas_pts_W.col(i) = meas_pt_W;
      }
    }
  }
  positive_meas_indices.resize(num_positive);
  positive_meas_pts_W.conservativeResize(NoChange, num_positive);

  // Compute correspondences (closest points in the point cloud and on the
  // tree's collision geometry).
  // TODO(eric.cousineau): Update once concave shapes are permitted.
  VectorXd positive_distances(num_positive);
  Matrix3Xd positive_normals_W(3, num_positive);  // world frame.
  Matrix3Xd positive_body_pts_W(3, num_positive);  // body point, world frame.
  Matrix3Xd positive_body_pts_Bi(3, num_positive);  // body point, body frame.
  vector<int> positive_body_indices(num_positive);
  {
    SCOPE_TIME(icp, "ICP Correspondence");
    mutable_tree.collisionDetectFromPoints(
          kin_cache, positive_meas_pts_W,
          positive_distances, positive_normals_W,
          positive_body_pts_W, positive_body_pts_Bi,
          positive_body_indices, use_margins);
  }

  // Bin each point based on the body ID.
  // Discard points that will not influence any
  vector<vector<int>> positive_body_point_indices(tree.get_num_bodies());
  for (int positive_index = 0; positive_index < num_positive; ++positive_index) {
    int body_index = positive_body_indices[positive_index];
    if (body_index > -1) {
      auto& body_positive_indices = positive_body_point_indices[body_index];
      body_positive_indices.push_back(positive_index);
    }
  }

  {
    SCOPE_TIME(icp, "ICP Costs");
    const double icp_weight = ComputeWeight(param_.icp.variance);
    IcpLinearizedNormAccumulator error_accumulator(tree.get_num_positions());
    // Go through each point and accumulate the cost.
    for (int body_index = 0; body_index < tree.get_num_bodies(); body_index++) {
      const auto& body_positive_indices = positive_body_point_indices[body_index];
      if (body_positive_indices.size() == 0) {
        continue;
      }
      const auto& body = tree.get_body(body_index);
      const RevoluteJoint* revolute_joint = nullptr;
      if (body_index > 0) {
        revolute_joint =
            dynamic_cast<const RevoluteJoint*>(&body.getJoint());
      }

      int used_points = 0;

      const int frame_Bi = body_index;
      // Accumulated points per body.
      IcpPointGroup icp_body(frame_Bi, body_positive_indices.size());

      for (int positive_index : body_positive_indices) {
        // Get coordinate from the measurement.
        bool use_point = false;
        Vector3d body_pt_Bi = positive_body_pts_Bi.col(positive_index);
        if (abs(positive_distances(positive_index)) <= param_.icp.max_distance_m) {
          // Ensure that this is far enough from the revolute joint axis.
          use_point = true;
          if (revolute_joint) {
            Vector3d joint_axis_B = revolute_joint->rotation_axis();
            double distance_to_axis = GetDistanceToAxis(body_pt_Bi, joint_axis_B);
            if (distance_to_axis < param_.icp.min_joint_distance_m) {
              use_point = false;
            }
          }
        }
        if (use_point) {
          used_points++;
          // Register each point with the appropriate frames to compute the
          // linearized error term.
          const int meas_index = positive_meas_indices[positive_index];
          icp_body.Add(positive_meas_pts_W.col(positive_index),
                       meas_pts_C.col(meas_index),
                       positive_body_pts_W.col(positive_index),
                       body_pt_Bi);
        }
      }
      // Incorporate into error accumulation.
      icp_body.AddTerms(scene, &error_accumulator, icp_weight);

      fmt::print(cout, "body: {}, points used: {}\n", body.get_name(),
                 used_points);
    }
    // Render to full cost, taking only the estimated variables.
    error_accumulator.UpdateCost(kin_est_slice.q(), icp_cost_.get());
  }
}

void DartDepthImageIcpObjective::DetermineUnaffectedBodies() {
  // TODO(eric.cousineau): Find easier way to do this. Presently, does not
  // appear to be an easy way to get instance id from tree index, and I have
  // to provide joint name and instance id for each kinematics item.
  // Should store these in the formulation in the future.
//  vector<int> influenced_bodies;
//  const auto& kin_slice = formulation().kinematics_est_slice();
//  const auto& q_slice = kin_slice.q();
//  for (int i = 0; i < q_slice.indices(); ++i) {
//    // Report the body for each joint.
//    tree.F
//    tree.FindChildBodyOfJoint();
//  }

//  // Compute which bodies will have no affect
//  // These will be used as a pseudo-segmentation mechanism.
//  vector<int> no_est_influence;
//  for (int body_index = 0; body_index < tree.get_num_bodies(); ++body_index) {
//    // Check which kinematic paths do not contain any of the estimated
//    // variables.
//    auto ancestors = tree.FindAncestorBodies(body_index);

//  }
}



}  // namespace manipulation
}  // namespace drake

#include "drake/manipulation/estimators/dev/dart_depth_image_icp_objective.h"

#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/common/scoped_timer.h"

#include <bot_lcmgl_client/lcmgl.h>
#include "drake/multibody/joints/revolute_joint.h"

using namespace drake::systems::sensors;

namespace drake {
namespace manipulation {

struct Coord {
  int u{0};  // x-pixel coordinate
  int v{0};  // y-pixel coordinate
};

/**
 * Fixed-width grid slice.
 */
// TODO(eric.cousineau): Support per-index mapping.
// TODO(eric.cousineau): Use better down-sampling (anti-aliasing, etc.). Look
// into using VTK for performing this.
class GridSlice {
 public:
  GridSlice(int downsample, int super_width, int super_height)
      : width_(super_width / downsample),
        height_(super_height / downsample),
        downsample_(downsample),
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
  void resize(int width, int height) {
    slice = GridSlice(width, height);
    depth_image.resize(width, height);
    point_cloud_C.resize(NoChange, slice.size());
  }
  void DownsampleFrom(int downsample, const Output& super) {
    slice = GridSlice(downsample, super.slice.width(), super.slice.height());
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
  void AddTerms(const Matrix3Xd& es, const MatrixXd& Jes) {
    int num_points = es.cols();
    for (int i = 0; i < num_points; ++i) {
      auto&& e = es.col(i);
      auto&& Je = Jes.middleRows(3 * i, 3);
      Q_ += 2 * Je.transpose() * Je;
      b_ += 2 * Je.transpose() * e;
      c_ = e.dot(e);
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
    cost->UpdateCoefficients(Q_, b_, c_);
  }
 private:
  MatrixXd Q_;
  VectorXd b_;
  double c_;
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

void DartDepthImageIcpObjective::Init(const KinematicsCached& cache) {
  unused(cache);
  prog().AddCost(icp_cost_, q_est_vars());
  prog().AddCost(free_space_cost_, q_est_vars());
}

void DartDepthImageIcpObjective::ObserveImage(
    double t, const ImageDepth32F& depth_image_meas,
    const Matrix3Xd* ppoint_cloud) {
  set_latest_observation_time(t);

  Impl& impl = *impl_;
  impl.meas_full.resize(depth_image_meas.width(), depth_image_meas.height());
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
  impl_->meas.DownsampleFrom(param_.image_downsample_factor, impl.meas_full);
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
  unused(obj_prior);
  KinematicsState kin_state(kin_cache);
  const bool use_margins = false;

  // Convenience aliases.
  Impl& impl = *impl_;
  const KinematicsSlice& kin_est_slice = formulation().kinematics_est_slice();
  const GridSlice& meas_slice = impl.meas.slice;
  // HACK(eric.cousineau): Mutable for `collisionDetectFromPoints()`.
  const RigidBodyTreed& tree = this->tree();
  RigidBodyTreed& mutable_tree = const_cast<RigidBodyTreed&>(tree);

  auto IsValidPoint = [this](auto&& pt) {
    if (isnan(pt[0])) {
      return false;
    } else {
      if (param_.point_cloud_bounds.IsInside(pt[0], pt[1], pt[2])) {
        return true;
      } else {
        return false;
      }
    }
  };

  // Identify points to be used for positive returns.
  // NOTE: Emphasis that `positive` is for positive RETURNS in the sense of
  // matching the to the point clouds (attracting), not positive signs.
  int num_meas = meas_slice.size();
  vector<int> positive_meas_indices(num_meas);  // Indices in `meas.slice`.
  Matrix3Xd positive_meas_pts_W(3, num_meas);  // Point cloud points.
  int num_positive = 0;

  const int frame_C = param_.camera.frame->get_frame_index();
  const int frame_W = tree.world().get_body_index();
  const Matrix3Xd& point_cloud_C = impl.meas.point_cloud_C;
  Matrix3Xd point_cloud_W =
      tree.transformPoints(kin_cache, point_cloud_C,
                           frame_C, frame_W);
  Coord c{};
  for (c.v = 0; c.v < meas_slice.height(); c.v++) {
    for (c.u = 0; c.u < meas_slice.width(); c.u++) {
      int meas_index = meas_slice.CoordToIndex(c);
      Vector3d pt_W = point_cloud_W.col(meas_index);
      if (IsValidPoint(pt_W)) {
        int i = num_positive++;
        positive_meas_indices[i] = meas_index;
        positive_meas_pts_W.col(i) = pt_W;
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
      auto& indices = positive_body_point_indices[body_index];
      indices.push_back(positive_index);
    }
  }

  IcpLinearizedNormAccumulator error_accumulator(tree.get_num_positions());
  // Go through each point and accumulate the cost.
  for (int body_index = 0; body_index < tree.get_num_bodies(); body_index++) {
    const auto& body = tree.get_body(body_index);
    const auto* revolute_joint =
        dynamic_cast<const RevoluteJoint*>(&body.getJoint());
    const auto& indices = positive_body_point_indices[body_index];

    const int frame_Bi = body_index;

    // Accumulated points per body.
    int num_body_points = indices.size();
    int num_icp_points = 0;
    Matrix3Xd icp_meas_pts_W(3, num_body_points);
    Matrix3Xd icp_meas_pts_C(3, num_body_points);
    Matrix3Xd icp_body_pts_Bi(3, num_body_points);
    Matrix3Xd icp_body_pts_W(3, num_body_points);

    for (int positive_index : indices) {
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
        // Register each point with the appropriate frames to compute the
        // linearized error term.
        const int i = num_icp_points++;
        icp_meas_pts_W.col(i) = positive_meas_pts_W.col(positive_index);
        const int meas_index = positive_meas_indices[positive_index];
        icp_meas_pts_C.col(i) = point_cloud_C.col(meas_index);
        icp_body_pts_Bi.col(i) = body_pt_Bi;
        icp_body_pts_W.col(i) = positive_body_pts_W.col(positive_index);
      }
    }
    // Trim down to complete set.
    auto trim_pts = [num_icp_points](auto&& X) {
      X.conservativeResize(NoChange, num_icp_points);
    };
    trim_pts(icp_meas_pts_W);
    trim_pts(icp_meas_pts_C);
    trim_pts(icp_body_pts_Bi);
    trim_pts(icp_body_pts_W);

    // Incorporate into linearized distance cost.
    // Get point jacobian w.r.t. camera frame, as that is the only influence
    // on the measured point cloud.
    MatrixXd J_meas_pts_W =
        tree.transformPointsJacobian(kin_cache, icp_meas_pts_C,
                                     frame_C, frame_W, false);
    MatrixXd J_body_pts_W =
        tree.transformPointsJacobian(kin_cache, icp_body_pts_Bi,
                                     frame_Bi, frame_W, false);
    // Compute errors.
    Matrix3Xd es_W =
        icp_meas_pts_W - icp_body_pts_W;
    MatrixXd J_es_W =
        J_meas_pts_W - J_body_pts_W;
    // Incorproate errors into L2 norm cost.
    error_accumulator.AddTerms(es_W, J_es_W);
  }
  // Render to full cost.
  error_accumulator.UpdateCost(kin_est_slice.q(), icp_cost_.get());
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

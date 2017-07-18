#pragma once

#include "drake/manipulation/estimators/dev/dart_icp.h
#include "drake/manipulation/estimators/dev/dart.h"
#include "drake/manipulation/estimators/dev/dart_objectives.h"

#include "drake/systems/sensors/image.h"

namespace drake {
namespace manipulation {

// Copied from RgbdCamera.
const int kImageWidth = 640;  // In pixels
const int kImageHeight = 480;  // In pixels

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

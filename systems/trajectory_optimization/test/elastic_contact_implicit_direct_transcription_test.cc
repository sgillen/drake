#include "drake/systems/trajectory_optimization/elastic_contact_implicit_direct_transcription.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
// #include "drake/lcm/drake_lcm.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree_construction.h"
// #include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/trajectory_optimization/rigid_body_tree_multiple_shooting_internal.h"
#include "drake/systems/trajectory_optimization/position_constraint_force_evaluator.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace {
// Construct a RigidBodyTree containing a four bar linkage.
std::unique_ptr<RigidBodyTree<double>>
ConstructContactImplicitBrickTree(bool is_empty) {
  RigidBodyTree<double>* tree = new RigidBodyTree<double>();
  const double plane_len = 100;
  multibody::AddFlatTerrainToWorld(tree, plane_len, plane_len);
  tree->a_grav << 0, 0, 0, 0, 0, -10;

  if (!is_empty) {
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        FindResourceOrThrow("drake/examples/contact_implicit_brick/contact_implicit_brick.urdf"),
        multibody::joints::kFixed, tree);
    DRAKE_DEMAND(tree->get_num_positions() == 1);
  }

  //RigidBody<double>* brick = tree->FindBody("contact_implicit_brick");
  //std::cerr<<brick->get_contact_points()<<std::endl;
  return std::unique_ptr<RigidBodyTree<double>>(tree);
}

GTEST_TEST(ElasticContactImplicitDirectTranscription, TestContactImplicitBrickNoContact) {
  auto tree = ConstructContactImplicitBrickTree(false);
  auto empty_tree = ConstructContactImplicitBrickTree(true);
  const int num_time_samples = 11;
  const double minimum_timestep{0.01};
  const double maximum_timestep{0.1};
  ElasticContactImplicitDirectTranscription traj_opt(
      *tree, *empty_tree, num_time_samples,
      minimum_timestep, maximum_timestep, 24, 0., 0.5);
  traj_opt.SetSolverOption(
      solvers::SnoptSolver::id(), "Print file", "/tmp/snopt.out");

  // Add a constraint on position 0 of the initial posture.
  double z_0 = 10;
  double z_f = 5;
  double z_f_margin = 0;
  double zdot_0_min = -25;
  double zdot_0_max = -0;
  unused(z_f);
  traj_opt.AddBoundingBoxConstraint(z_0, z_0,
                                    traj_opt.GeneralizedPositions()(0, 0));

  // Add a constraint on velocity 0 of the initial posture.
  traj_opt.AddBoundingBoxConstraint(zdot_0_min, zdot_0_max,
                                    traj_opt.GeneralizedVelocities()(0, 0));

  // const int N = num_time_samples;
  // traj_opt.AddBoundingBoxConstraint(-10, -10,
  //                                   traj_opt.GeneralizedVelocities()(0, N - 1));


  traj_opt.AddBoundingBoxConstraint(1, 1,
                                    traj_opt.GeneralizedPositions()(0, 5));

  // traj_opt.AddBoundingBoxConstraint(0, 0, traj_opt.ContactConstraintForces());

  // Add a constraint on the final posture.
  traj_opt.AddBoundingBoxConstraint(
      z_f - z_f_margin, z_f + z_f_margin,
      traj_opt.GeneralizedPositions()(0, num_time_samples - 1));

  // Add a constraint on the final velocity.
  //traj_opt.AddBoundingBoxConstraint(
      //-15, -1, traj_opt.GeneralizedVelocities().col(0));
  // Add a running cost on the control as ∫ v² dt.
  traj_opt.AddRunningCost(
      traj_opt.GeneralizedVelocities().cast<symbolic::Expression>().squaredNorm());

  // Add direct transcription constraints.
  traj_opt.Compile();
  const solvers::SolutionResult result = traj_opt.Solve();

  EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound);

  const double tol{1E-5};
  // First check if dt is within the bounds.
  const Eigen::VectorXd t_sol = traj_opt.GetSampleTimes();
  const Eigen::VectorXd dt_sol =
      t_sol.tail(num_time_samples - 1) - t_sol.head(num_time_samples - 1);
  EXPECT_TRUE(
      (dt_sol.array() <=
       Eigen::ArrayXd::Constant(num_time_samples - 1, maximum_timestep + tol))
          .all());
  EXPECT_TRUE(
      (dt_sol.array() >=
       Eigen::ArrayXd::Constant(num_time_samples - 1, minimum_timestep - tol))
          .all());
  // Check if the interpolation constraint is satisfied
  KinematicsCache<double> kinsol = tree->CreateKinematicsCache();
  const Eigen::MatrixXd q_sol =
      traj_opt.GetSolution(traj_opt.GeneralizedPositions());
  const Eigen::MatrixXd v_sol =
      traj_opt.GetSolution(traj_opt.GeneralizedVelocities());
  Eigen::MatrixXd u_sol(tree->get_num_actuators(), num_time_samples);
  const Eigen::MatrixXd lambda_sol =
      traj_opt.GetSolution(traj_opt.ContactConstraintForces());
  for (int i = 0; i < num_time_samples; ++i) {
    u_sol.col(i) = traj_opt.GetSolution(traj_opt.input(i));
  }

  std::cerr<<"Q SOL"<<std::endl;
  std::cerr<<q_sol<<std::endl;
  std::cerr<<"V SOL"<<std::endl;
  std::cerr<<v_sol<<std::endl;
  std::cerr<<"T SOL"<<std::endl;
  std::cerr<<t_sol.transpose()<<std::endl;
  std::cerr<<"L SOL"<<std::endl;
  std::cerr<<lambda_sol<<std::endl;


  for (int i = 1; i < num_time_samples; ++i) {
    int v_dyn = v_sol.col(i).rows()/2;
    Eigen::VectorXd v_sol_kin = v_sol.col(i).tail(v_dyn);
    Eigen::VectorXd v_sol_kin_old = v_sol.col(i-1).head(v_dyn);
    DRAKE_ASSERT(v_sol_kin.rows() == v_sol_kin_old.rows());
    kinsol.initialize(q_sol.col(i), v_sol_kin);
    tree->doKinematics(kinsol, true);

    // Check qᵣ - qₗ = q̇ᵣ*h
    EXPECT_TRUE(CompareMatrices(q_sol.col(i) - q_sol.col(i - 1),
                                v_sol_kin * dt_sol(i - 1), tol,
                                MatrixCompareType::absolute));

    // Check Mᵣ(vᵣ - vₗ) = (B*uᵣ + Jᵣᵀ*λᵣ -c(qᵣ, vᵣ))h
    const Eigen::MatrixXd M = tree->massMatrix(kinsol);
    const Eigen::MatrixXd J_r = tree->positionConstraintsJacobian(kinsol);
    const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
    const Eigen::VectorXd c =
        tree->dynamicsBiasTerm(kinsol, no_external_wrenches);
    EXPECT_TRUE(CompareMatrices(
        M * (v_sol_kin - v_sol_kin_old),
        (tree->B * u_sol.col(i) - c) *
            dt_sol(i - 1),
        tol, MatrixCompareType::relative));
  }
  // Check if the constraints on the initial state and final state are
  // satisfied.
  EXPECT_NEAR(q_sol(0, 0), z_0, tol);
  // EXPECT_NEAR(q_sol(0, num_time_samples - 1), z_f, tol);
  // EXPECT_TRUE(CompareMatrices(v_sol.col(num_time_samples - 1),
  //                             Eigen::VectorXd::Zero(tree->get_num_velocities()),
  //                             tol, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake

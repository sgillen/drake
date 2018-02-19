#include "drake/systems/trajectory_optimization/rigid_body_tree_multiple_shooting.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/trajectory_optimization/rigid_body_tree_multiple_shooting_internal.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace {
// Construct a RigidBodyTree containing a four bar linkage.
std::unique_ptr<RigidBodyTree<double>> ConstructContactImplicitBrickTree() {
  RigidBodyTree<double>* tree = new RigidBodyTree<double>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/contact_implicit_brick/contact_implicit_brick.urdf"),
      multibody::joints::kFixed, tree);
  std::cout << "DEBUG: tree->get_num_positions() = " << tree->get_num_positions() << std::endl;
  DRAKE_DEMAND(tree->get_num_positions() == 1);
//  DRAKE_DEMAND(tree->get_num_actuators() == 1);
  return std::unique_ptr<RigidBodyTree<double>>(tree);
}

GTEST_TEST(DirectTranscriptionConstraintTest, TestEval) {
  // Test the evaluation of DirectTranscriptionConstraintTest
  auto tree = ConstructContactImplicitBrickTree();
  const int num_lambda = tree->getNumPositionConstraints();
  auto kinematics_helper =
      std::make_shared<plants::KinematicsCacheWithVHelper<AutoDiffXd>>(*tree);

  auto position_constraint_force_evaluator =
      std::make_unique<PositionConstraintForceEvaluator>(*tree,
                                                         kinematics_helper);

  DirectTranscriptionConstraint constraint(*tree, kinematics_helper);

  constraint.AddGeneralizedConstraintForceEvaluator(
      std::move(position_constraint_force_evaluator));

  // Set q, v, u, lambda to arbitrary values.
  const double h = 0.1;
  const Eigen::VectorXd q_l =
      Eigen::VectorXd::LinSpaced(tree->get_num_positions(), 1, 1);
  const Eigen::VectorXd v_l =
      Eigen::VectorXd::LinSpaced(tree->get_num_velocities(), 0, 2);
  const Eigen::VectorXd q_r =
      Eigen::VectorXd::LinSpaced(tree->get_num_positions(), 1, 1);
  const Eigen::VectorXd v_r =
      Eigen::VectorXd::LinSpaced(tree->get_num_velocities(), -2, 3);
  const Eigen::VectorXd u_r =
      Eigen::VectorXd::LinSpaced(tree->get_num_actuators(), 2, 3);
  const Eigen::VectorXd lambda_r = Eigen::VectorXd::LinSpaced(num_lambda, 3, 5);

  const Eigen::VectorXd x =
      constraint.CompositeEvalInput(h, q_l, v_l, q_r, v_r, u_r, lambda_r);
  const AutoDiffVecXd tx = math::initializeAutoDiff(x);
  AutoDiffVecXd ty;
  constraint.Eval(tx, ty);

  Eigen::VectorXd y_expected(tree->get_num_positions() +
                             tree->get_num_velocities());
  y_expected.head(tree->get_num_positions()) = q_r - q_l - v_r * h;
  KinematicsCache<double> kinsol = tree->CreateKinematicsCache();
  kinsol.initialize(q_r, v_r);
  tree->doKinematics(kinsol, true);
  const Eigen::MatrixXd M = tree->massMatrix(kinsol);
  const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  const Eigen::VectorXd c =
      tree->dynamicsBiasTerm(kinsol, no_external_wrenches);
  const Eigen::MatrixXd J = tree->positionConstraintsJacobian(kinsol);
  y_expected.tail(tree->get_num_velocities()) =
      M * (v_r - v_l) - (tree->B * u_r + J.transpose() * lambda_r - c) * h;
  EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(ty), y_expected,
                              1E-10, MatrixCompareType::absolute));
}

GTEST_TEST(RigidBodyTreeMultipleShootingTest, TestSimpleContactImplicitBrick) {
  auto tree = ConstructContactImplicitBrickTree();
  const int num_time_samples = 10;
  const double minimum_timestep{0.01};
  const double maximum_timestep{0.1};
  RigidBodyTreeMultipleShooting traj_opt(*tree, num_time_samples,
                                         minimum_timestep, maximum_timestep);


  // Add a constraint on position 0 of the initial posture.
  traj_opt.AddBoundingBoxConstraint(1, 1,
                                    traj_opt.GeneralizedPositions()(0, 0));
  // Add a constraint on the final posture.
  traj_opt.AddBoundingBoxConstraint(
      0.5, 0.5, traj_opt.GeneralizedPositions()(0, num_time_samples - 1));
  // Add a constraint on the final velocity.
  traj_opt.AddBoundingBoxConstraint(
      -5, 5, traj_opt.GeneralizedVelocities().col(num_time_samples - 1));
  // Add a running cost on the control as ∫ u² dt.
  traj_opt.AddRunningCost(
      traj_opt.input().cast<symbolic::Expression>().squaredNorm());

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
      traj_opt.GetSolution(traj_opt.PositionConstraintForces());
  for (int i = 0; i < num_time_samples; ++i) {
    u_sol.col(i) = traj_opt.GetSolution(traj_opt.input(i));
  }

  for (int i = 1; i < num_time_samples; ++i) {
    kinsol.initialize(q_sol.col(i), v_sol.col(i));
    tree->doKinematics(kinsol, true);
    // Check qᵣ - qₗ = q̇ᵣ*h
    EXPECT_TRUE(CompareMatrices(q_sol.col(i) - q_sol.col(i - 1),
                                v_sol.col(i) * dt_sol(i - 1), tol,
                                MatrixCompareType::absolute));
    // Check Mᵣ(vᵣ - vₗ) = (B*uᵣ + Jᵣᵀ*λᵣ -c(qᵣ, vᵣ))h
    const Eigen::MatrixXd M = tree->massMatrix(kinsol);
    const Eigen::MatrixXd J_r = tree->positionConstraintsJacobian(kinsol);
    const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
    const Eigen::VectorXd c =
        tree->dynamicsBiasTerm(kinsol, no_external_wrenches);
    EXPECT_TRUE(CompareMatrices(
        M * (v_sol.col(i) - v_sol.col(i - 1)),
        (tree->B * u_sol.col(i) + J_r.transpose() * lambda_sol.col(i) - c) *
            dt_sol(i - 1),
        tol, MatrixCompareType::relative));
  }
  // Check if the constraints on the initial state and final state are
  // satisfied.
  EXPECT_NEAR(q_sol(0, 0), 1, tol);
  EXPECT_NEAR(q_sol(0, num_time_samples - 1), 0.5, tol);
  // EXPECT_TRUE(CompareMatrices(v_sol.col(num_time_samples - 1),
  //                             Eigen::VectorXd::Zero(tree->get_num_velocities()),
  //                             tol, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake

#include "drake/systems/trajectory_optimization/rigid_body_tree_trajectory_optimization.h"
#include "drake/systems/trajectory_optimization/rigid_body_tree_trajectory_optimization_internal.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace {
// Construct a RigidBodyTree containing the contact implicit brkck.
std::unique_ptr<RigidBodyTree<double>> ConstructContactImplicitBrick() {
  RigidBodyTree<double>* tree = new RigidBodyTree<double>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/contact_implicit_brick/contact_implicit_brick.urdf"),
      multibody::joints::kRollPitchYaw, tree);
  return std::unique_ptr<RigidBodyTree<double>>(tree);
}

GTEST_TEST(GeneralizedConstraintForceEvaluatorTest, TestEval) {
  // Test the Eval function of GeneralizedConstraintForceEvaluator.
  auto tree = ConstructFourBarTree();
  const int num_lambda = tree->getNumPositionConstraints();

  auto cache_helper =
      std::make_shared<KinematicsCacheWithVHelper<AutoDiffXd>>(*tree);

  GeneralizedConstraintForceEvaluator evaluator(*tree, num_lambda,
                                                cache_helper);

  // Set q to some arbitrary number.
  Eigen::VectorXd q(tree->get_num_positions());
  for (int i = 0; i < q.rows(); ++i) {
    q(i) = i + 1;
  }
  // Set lambda to some arbitrary number.
  Eigen::VectorXd lambda(num_lambda);
  for (int i = 0; i < lambda.rows(); ++i) {
    lambda(i) = 2 * i + 3;
  }
  Eigen::VectorXd x(q.rows() + lambda.rows());
  x << q, lambda;
  const auto tx = math::initializeAutoDiff(x);
  AutoDiffVecXd ty;
  evaluator.Eval(tx, ty);
  EXPECT_EQ(ty.rows(), tree->get_num_velocities());
  KinematicsCache<double> kinsol = tree->CreateKinematicsCache();
  kinsol.initialize(q);
  tree->doKinematics(kinsol);
  const auto J = tree->positionConstraintsJacobian(kinsol);
  const Eigen::VectorXd y_expected = J.transpose() * lambda;
  EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(ty), y_expected,
                              1E-10, MatrixCompareType::absolute));
}

GTEST_TEST(DirectTranscriptionConstraintTest, TestEval) {
  // Test the evaluation of DirectTranscriptionConstraintTest
  auto tree = ConstructContactImplicitBrick();
  const int num_lambda = tree->getNumPositionConstraints();
  auto kinematics_helper =
      std::make_shared<KinematicsCacheWithVHelper<AutoDiffXd>>(*tree);

  DirectTranscriptionConstraint constraint(*tree, num_lambda,
                                           kinematics_helper);

  // Set q, v, u, lambda to arbitrary values.
  const double h = 0.1;
  const Eigen::VectorXd q_l =
      Eigen::VectorXd::LinSpaced(tree->get_num_positions(), 0, 1);
  const Eigen::VectorXd v_l =
      Eigen::VectorXd::LinSpaced(tree->get_num_velocities(), 0, 2);
  const Eigen::VectorXd q_r =
      Eigen::VectorXd::LinSpaced(tree->get_num_positions(), -1, 1);
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

}  // namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
#include "drake/systems/trajectory_optimization/elastic_contact_implicit_direct_transcription.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/trajectory_optimization/rigid_body_tree_multiple_shooting_internal.h"
#include "drake/systems/trajectory_optimization/position_constraint_force_evaluator.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace {


using std::unique_ptr;
using ::drake::lcm::DrakeLcm;

using T = double;

class HackViz {
 public:
  HackViz(const RigidBodyTree<T>& tree)
      : tree_(&tree), system_(*tree_, &lcm_, true) {
    context_ = system_.CreateDefaultContext();
    nq_ = tree_->get_num_positions();
    nv_ = tree_->get_num_velocities();
    const VectorX<T> x0 = VectorX<T>::Zero(nq_ + nv_);
    input_value_ = &context_->FixInputPort(0, x0);
  }

  void Update(double t, const VectorX<T>& q) {
    context_->set_time(t);
    auto&& x = input_value_->GetMutableVectorData<T>()->get_mutable_value();
    x.head(nq_) = q;
    if (!is_inited_) {
      // Copied from simulator.
      auto init_events = system_.AllocateCompositeEventCollection();
      system_.GetInitializationEvents(*context_, init_events.get());
      system_.Publish(*context_, init_events->get_publish_events());
      is_inited_ = true;
    } else {
      system_.Publish(*context_);
    }
  }

  void ReplayCachedSimulation() {
    system_.ReplayCachedSimulation();
  }

 private:
  const RigidBodyTree<T>* tree_{};
  int nq_{};
  int nv_{};
  DrakeLcm lcm_;
  DrakeVisualizer system_;
  FreestandingInputPortValue* input_value_{};
  unique_ptr<Context<T>> context_;
  bool is_inited_{false};
};


// Construct a RigidBodyTree containing a four bar linkage.
std::unique_ptr<RigidBodyTree<double>>
ConstructContactImplicitBrickTree(
    bool is_2d, bool is_point_mass, bool is_empty) {
  RigidBodyTree<double>* tree = new RigidBodyTree<double>();
  const double plane_len = 100;
  multibody::AddFlatTerrainToWorld(tree, plane_len, plane_len);
  tree->a_grav << 0, 0, 0, 0, 0, -10;

  if (!is_empty) {
    std::string prefix = "drake/examples/contact_implicit_brick/";
    std::string model = is_point_mass ? "point_mass" : "contact_implicit_brick";
    std::string dim = is_2d ? "2d" : "1d";
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        FindResourceOrThrow(prefix + model + "_" + dim + ".urdf"),
        multibody::joints::kFixed, tree);
  }

  //RigidBody<double>* brick = tree->FindBody("contact_implicit_brick");
  //std::cerr<<brick->get_contact_points()<<std::endl;
  return std::unique_ptr<RigidBodyTree<double>>(tree);
}

GTEST_TEST(ElasticContactImplicitDirectTranscription,
    DISABLED_TestContactImplicitBrickNoContact) {
  const bool is_2d = true;
  const bool is_point_mass = false;
  auto tree =
      ConstructContactImplicitBrickTree(is_2d, is_point_mass, false);
  auto empty_tree =
      ConstructContactImplicitBrickTree(is_2d, is_point_mass, true);
  const int num_time_samples = 11;
  const int N = num_time_samples;
  const double minimum_timestep{0.05};
  const double maximum_timestep{0.1};

  RigidBody<double>* brick = tree->FindBody("contact_implicit_brick");
  const auto& contacts_B = brick->get_contact_points();
  const int num_contacts = contacts_B.cols();

  const double comp_tol = 0.;
  const double elasticity = 0.5;
  ElasticContactImplicitDirectTranscription traj_opt(
      *tree, *empty_tree, num_time_samples,
      minimum_timestep, maximum_timestep, num_contacts,
      comp_tol, elasticity);
  traj_opt.SetSolverOption(
      solvers::SnoptSolver::id(), "Print file", "/tmp/snopt.out");

  // Add a constraint on position 0 of the initial posture.
  double z_0 = 10;
  double z_f = 5;
  double z_f_margin = 0;
  double zdot_0_min = -50;
  double zdot_0_max = -1;

  int ix = 0;
  int iz = 1;
  if (!is_2d) {
    ix = -1;
    iz = 0;
    DRAKE_DEMAND(tree->get_num_positions() == 1);
  } else {
    DRAKE_DEMAND(tree->get_num_positions() == 2);
  }
  unused(ix);

  unused(z_f);
  traj_opt.AddBoundingBoxConstraint(z_0, z_0,
                                    traj_opt.GeneralizedPositions()(iz, 0));

  // Add a constraint on velocity 0 of the initial posture.
  traj_opt.AddBoundingBoxConstraint(zdot_0_min, zdot_0_max,
                                    traj_opt.GeneralizedVelocities()(iz, 0));

  double x_0 = -1;
  double x_f = 1;
  if (is_2d) {
    traj_opt.AddBoundingBoxConstraint(x_0, x_0,
                                      traj_opt.GeneralizedPositions()(ix, 0));
    traj_opt.AddBoundingBoxConstraint(x_f, x_f,
                                      traj_opt.GeneralizedPositions()(ix, N - 1));
  }

  // Seed initial guess.
  for (int i = 0; i < N; ++i) {
    if (is_2d) {
      auto xi = traj_opt.GeneralizedPositions()(ix, i);
      traj_opt.SetInitialGuess(xi, x_0 + (x_f - x_0) * i / (N - 1));
    }
    auto zi = traj_opt.GeneralizedPositions()(iz, i);
    auto zdi = traj_opt.GeneralizedVelocities()(iz, i);
    if (N < 5) {
      traj_opt.SetInitialGuess(zi, 10 - 2*i);
      traj_opt.SetInitialGuess(zdi, -5);
    } else {
      traj_opt.SetInitialGuess(zi, 2*(i - 5));
      traj_opt.SetInitialGuess(zdi, 2*(i - 5));
    }
  }

  traj_opt.AddBoundingBoxConstraint(0, 5,
                                    traj_opt.GeneralizedPositions()(iz, 5));

  // traj_opt.AddBoundingBoxConstraint(0, 0, traj_opt.ContactConstraintForces());

  // Add a constraint on the final posture.
  traj_opt.AddBoundingBoxConstraint(
      z_f - z_f_margin, z_f + z_f_margin,
      traj_opt.GeneralizedPositions()(iz, num_time_samples - 1));

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
    // v = [vplus; vminus]
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
  EXPECT_NEAR(q_sol(iz, 0), z_0, tol);
  // EXPECT_NEAR(q_sol(0, num_time_samples - 1), z_f, tol);
  // EXPECT_TRUE(CompareMatrices(v_sol.col(num_time_samples - 1),
  //                             Eigen::VectorXd::Zero(tree->get_num_velocities()),
  //                             tol, MatrixCompareType::absolute));


  HackViz viz(*tree);
  const double dt_anim = 0.5;
  for (int i = 0; i < N; ++i) {
    viz.Update(dt_anim * i, q_sol.col(i));
  }

  while (true) {
    viz.ReplayCachedSimulation();
  }

}

// Construct a RigidBodyTree containing a four bar linkage.
std::unique_ptr<RigidBodyTree<double>>
ConstructBasketCase(bool is_empty) {
  RigidBodyTree<double>* tree = new RigidBodyTree<double>();
  const double plane_len = 100;
  multibody::AddFlatTerrainToWorld(tree, plane_len, plane_len);
  tree->a_grav << 0, 0, 0, 0, 0, -10;

  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        FindResourceOrThrow(
            "drake/examples/contact_implicit_brick/basket_case_scene.urdf"),
        multibody::joints::kFixed, tree);

  if (!is_empty) {
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        FindResourceOrThrow(
            "drake/examples/contact_implicit_brick/contact_implicit_brick_2d.urdf"),
        multibody::joints::kFixed, tree);
    std::cout << "npos: " << tree->get_num_positions() << std::endl;
    DRAKE_DEMAND(tree->get_num_positions() == 2);
  }

  return std::unique_ptr<RigidBodyTree<double>>(tree);
}


GTEST_TEST(ElasticContactImplicitDirectTranscription,
    TestBasketCase) {
  auto tree = ConstructBasketCase(false);
  auto empty_tree = ConstructBasketCase(true);
  const int num_time_samples = 21;
  const int N = num_time_samples;
  const double minimum_timestep{0.1};
  const double maximum_timestep{0.2};

  const double comp_tol = 0.;
  const double elasticity = 0.1;

  RigidBody<double>* brick = tree->FindBody("contact_implicit_brick");
  const auto& contacts_B = brick->get_contact_points();
  const int num_contacts = contacts_B.cols();

  ElasticContactImplicitDirectTranscription traj_opt(
      *tree, *empty_tree, num_time_samples,
      minimum_timestep, maximum_timestep, num_contacts, comp_tol, elasticity);
  traj_opt.SetSolverOption(
      solvers::SnoptSolver::id(), "Print file", "/tmp/snopt.out");

  const int ix = 0;
  const int iz = 1;
  const int nv = tree->get_num_velocities();

  int np0 = 3;
  Eigen::Matrix2Xd p0s(2, np0);
  p0s.transpose() <<
      1, 1,
      19, 25,
      15, 1;
  // Seed initial guess interpolated along waypoints.
  for (int i = 0; i < N; ++i) {
    int ip_left = i * (np0 - 1) / (N - 1);
    if (ip_left == np0 - 1)
      ip_left -= 1;
    int ip_right = ip_left + 1;

    int i_left = ip_left * (N - 1) / (np0 - 1);
    int i_right = ip_right * (N - 1) / (np0 - 1);
    double s = (i - i_left) / float(i_right - i_left);
    auto p_left = p0s.col(ip_left);
    auto p_right = p0s.col(ip_right);
    auto p = p_left + s * (p_right - p_left);
    const double dt_est = 0.1;
    auto pd = (p_right - p_left) / (dt_est * i_right - i_left);

    auto xi = traj_opt.GeneralizedPositions()(ix, i);
    auto zi = traj_opt.GeneralizedPositions()(iz, i);
    auto xdi = traj_opt.GeneralizedVelocities()(ix, i);
    auto zdi = traj_opt.GeneralizedVelocities()(iz, i);
    auto xdi_minus = traj_opt.GeneralizedVelocities()(ix + nv, i);
    auto zdi_minus = traj_opt.GeneralizedVelocities()(iz + nv, i);
    traj_opt.SetInitialGuess(xi, p(ix));
    traj_opt.SetInitialGuess(zi, p(iz));
    traj_opt.SetInitialGuess(xdi, pd(ix));
    traj_opt.SetInitialGuess(zdi, pd(iz));
    traj_opt.SetInitialGuess(xdi_minus, pd(ix));
    traj_opt.SetInitialGuess(zdi_minus, pd(iz));
  }

  traj_opt.AddBoundingBoxConstraint(
      1, 1, traj_opt.GeneralizedPositions().col(0));
  traj_opt.AddBoundingBoxConstraint(
      0, 30, traj_opt.GeneralizedVelocities()(ix, 0));
  traj_opt.AddBoundingBoxConstraint(
      0, 30, traj_opt.GeneralizedVelocities()(iz, 0));

  traj_opt.AddBoundingBoxConstraint(
      13, 18, traj_opt.GeneralizedPositions()(ix, N-1));
  traj_opt.AddBoundingBoxConstraint(
      1, 1, traj_opt.GeneralizedPositions()(iz, N-1));

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

  const Eigen::MatrixXd q_i =
      traj_opt.GetInitialGuess(traj_opt.GeneralizedPositions());
  std::cerr<<"Q I"<<std::endl;
  std::cerr<<q_i<<std::endl;

  const Eigen::MatrixXd v_i =
      traj_opt.GetInitialGuess(traj_opt.GeneralizedVelocities());
  std::cerr<<"V I"<<std::endl;
  std::cerr<<v_i<<std::endl;

  HackViz viz(*tree);
  const double dt_anim = 0.5;
  for (int i = 0; i < N; ++i) {
    viz.Update(dt_anim * i, q_sol.col(i));
    // viz.Update(dt_anim * i, q_i.col(i));
  }

  while (true) {
    viz.ReplayCachedSimulation();
  }

  // // Check if the constraints on the initial state and final state are
  // // satisfied.
  // EXPECT_NEAR(q_sol(0, 0), z_0, tol);
}



}  // namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake

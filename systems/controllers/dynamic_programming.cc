#include "drake/systems/controllers/dynamic_programming.h"

#include <limits>
#include <utility>
#include <vector>

#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace systems {
namespace controllers {

std::pair<std::unique_ptr<BarycentricMeshSystem<double>>, Eigen::MatrixXd>
FittedValueIteration(
    Simulator<double>* simulator,
    const std::function<double(const Context<double>& context)>& cost_function,
    const math::BarycentricMesh<double>::MeshGrid& state_grid,
    const math::BarycentricMesh<double>::MeshGrid& input_grid,
    const double timestep, const DynamicProgrammingOptions& options) {
  // TODO(russt): handle discrete state.
  const auto& system = simulator->get_system();
  auto& context = simulator->get_mutable_context();

  DRAKE_DEMAND(context.has_only_continuous_state());
  DRAKE_DEMAND(context.get_continuous_state().size() ==
               static_cast<int>(state_grid.size()));

  DRAKE_DEMAND(context.get_num_input_ports() == 1);
  DRAKE_DEMAND(system.get_num_total_inputs() ==
               static_cast<int>(input_grid.size()));

  DRAKE_DEMAND(timestep > 0.);
  DRAKE_DEMAND(options.discount_factor > 0. && options.discount_factor <= 1.);
  if (!options.state_indices_with_periodic_boundary_conditions.empty()) {
    // Make sure all periodic boundary conditions are in range.
    DRAKE_DEMAND(
        *options.state_indices_with_periodic_boundary_conditions.begin() >= 0);
    DRAKE_DEMAND(
        *options.state_indices_with_periodic_boundary_conditions.rbegin() <
        context.get_continuous_state().size());
  }

  // TODO(russt): check that the system is time-invariant.

  math::BarycentricMesh<double> state_mesh(state_grid);
  math::BarycentricMesh<double> input_mesh(input_grid);

  const int kNumStates = state_mesh.get_num_mesh_points();
  const int kNumInputs = input_mesh.get_num_mesh_points();
  const int kNumIndices = state_mesh.get_num_interpolants();

  std::vector<Eigen::MatrixXi> Tind(kNumInputs);
  std::vector<Eigen::MatrixXd> T(kNumInputs);
  std::vector<Eigen::RowVectorXd> cost(kNumInputs);

  {  // Build transition matrices.
    std::cout << "Computing transition and cost matrices";
    auto& sim_state = context.get_mutable_continuous_state_vector();

    Eigen::VectorXd input_vec(input_mesh.get_input_size());
    Eigen::VectorXd state_vec(state_mesh.get_input_size());

    Eigen::VectorXi Tind_tmp(kNumIndices);
    Eigen::VectorXd T_tmp(kNumIndices);

    for (int input = 0; input < kNumInputs; input++) {
      std::cout << ".";
      Tind[input].resize(kNumIndices, kNumStates);
      T[input].resize(kNumIndices, kNumStates);
      cost[input].resize(kNumStates);

      input_mesh.get_mesh_point(input, &input_vec);
      context.FixInputPort(0, input_vec);

      for (int state = 0; state < kNumStates; state++) {
        context.set_time(0.0);
        sim_state.SetFromVector(state_mesh.get_mesh_point(state));

        cost[input](state) = timestep * cost_function(context);

        simulator->StepTo(timestep);
        state_vec = sim_state.CopyToVector();

        for (int dim :
             options.state_indices_with_periodic_boundary_conditions) {
          const double lower = *state_grid[dim].begin();
          const double upper = *state_grid[dim].rbegin();
          state_vec[dim] =
              std::fmod(state_vec[dim] - lower, upper - lower) + lower;
        }

        state_mesh.EvalBarycentricWeights(state_vec, &Tind_tmp, &T_tmp);
        Tind[input].col(state) = Tind_tmp;
        T[input].col(state) = T_tmp;
      }
    }
    std::cout << "done." << std::endl;
  }

  // Perform value iteration loop
  Eigen::RowVectorXd J = Eigen::RowVectorXd::Zero(kNumStates);
  Eigen::RowVectorXd Jnext(kNumStates);
  Eigen::RowVectorXi Pi(kNumStates);

  double max_diff = std::numeric_limits<double>::infinity();
  while (max_diff > options.convergence_tol) {
    for (int state = 0; state < kNumStates; state++) {
      Jnext(state) = std::numeric_limits<double>::infinity();

      for (int input = 0; input < kNumInputs; input++) {
        double Jinput = cost[input](state);
        for (int index = 0; index < kNumIndices; index++) {
          Jinput += options.discount_factor * T[input](index, state) *
                    J(Tind[input](index, state));
        }
        if (Jinput < Jnext(state)) {
          Jnext(state) = Jinput;
          Pi(state) = input;
        }
      }
    }
    max_diff = (J - Jnext).lpNorm<Eigen::Infinity>();
    J = Jnext;
    //    std::cout << "J  = " << J << std::endl;
    //    std::cout << "Pi = " << Pi << std::endl;
  }

  // Create the policy.
  Eigen::MatrixXd policy_values(input_mesh.get_input_size(), kNumStates);
  for (int state = 0; state < kNumStates; state++) {
    policy_values.col(state) = input_mesh.get_mesh_point(Pi(state));
  }
  auto policy = std::make_unique<BarycentricMeshSystem<double>>(state_mesh,
                                                                policy_values);
  return std::make_pair(std::move(policy), J);
}

}  // namespace controllers
}  // namespace systems
}  // namespace drake

/** @file
 Provides sugar methods to construct a MultibodyPlant and SceneGraph, build
 them into the same diagram,
 functions to create a Diagram from a MultibodyPlant and a SceneGraph,
 allowing this diagram to be simulated (in time) and geometric queries to be
 posed, e.g., for planning applications.
*/

#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace multibody_plant {

template <typename T>
struct PlantContextPair {
  /// Plant context.
  Context<T>* context{};
  /// Owned context; may be the same as `context`, or may be its parent context.
  std::unique_ptr<T> parent_context;
};

template <typename T>
PlantContextPair<T> CreateDefaultContext(
    const MultibodyPlant<T>& plant, const Diagram<T>* diagram = nullptr) {
  DRAKE_DEMAND(plant.is_finalized());
  if (diagram) {
    auto parent_context = diagram->CreateDefaultContext();
    auto* context = diagram->GetMutableSubsystemContext(&plant);
    return PlantContextPair<T>{context, std::move(parent_context)};
  } else {
    auto parent_context = plant.CreateDefaultContext();
    return PlantContextPair<T>{parent_context.get(), std::move(parent_context)};
  }
}

template <typename T>
struct MultibodyWorld {
  MultibodyPlant<T>* plant{};
  geometry::SceneGraph<T>* scene_graph{};
  std::unique_ptr<Diagram<T>> diagram;

  void FinalizeAndBuild(DiagramBuilder* builder) {
    plant->Finalize(scene_graph);
    diagram = builder->Build();
  }

  PlantContextPair<T> CreateDefaultContext() const {
    return CreateDefaultContext(*plant, diagram.get());
  }

  bool SupportsGeometry() const {
    return scene_graph != nullptr && diagram != nullptr;
  }
}

template <typename T>
MultibodyWorld<T> CreatePlantAndSceneGraph(
    DiagramBuilder<T>* builder, double period = 0.) {
  MultibodyWorld world;
  world.plant = builder->AddSystem<MultibodyPlant>(period);
  world.plant->set_name("plant");
  world.scene_graph = builder->AddSystem<geometry::SceneGraph>();
  world.scene_graph->set_name("scene_graph");
  world.plant->RegisterAsSourceForSceneGraph(world.scene_graph);
  builder->Connect(
      world.plant->get_geometry_poses_output_port(),
      world.scene_graph->get_source_pose_port(
          world.plant->get_source_id().value()));
  builder->Connect(
      world.scene_graph->get_query_output_port(),
      world.plant->get_geometry_query_input_port());
  return std::move(world);
}

}  // namespace multibody_plant
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake

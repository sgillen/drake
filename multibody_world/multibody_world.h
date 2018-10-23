/** @file
 Provides functions to create a Diagram from a MultibodyPlant and a SceneGraph,
 allowing this diagram to be simulated (in time) and geometric queries to be
 posed, e.g., for planning applications.
*/

#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

namespace drake {

/// @cond
// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_MBW_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)
/// @endcond

/** Extends a Diagram in order to wrap and connect a MultibodyPlant and a
 SceneGraph, thereby allowing users to easily pose geometric queries for
 arbitrary multibody configurations. All ports from SceneGraph and
 MultibodyPlant are exported for convenience. Functionality for connecting
 to DrakeVisualizer is also provided.

 Like MultibodyPlant, MultibodyWorld must be constructed in two
 phases: construction and finalization. This two-phase construction is necessary
 so that the MultibodyPlant can be initialized in the requisite manner.
 Connection to DrakeVisualizer requires yet another step. A sketch of code for
 the typical process follows:
 @code
 systems::DiagramBuilder<double> builder;

 auto& mbw = *builder.AddSystem<MultibodyWorld<double>>();
 auto& plant = mbw.mutable_multibody_plant();

 // Make and add the cart_pole model.
 AddModelFromSdfFile(filename, &plant, &mbw.mutable_scene_graph());

 // Add gravity to the model.
 plant.AddForceElement<UniformGravityFieldElement>(
     -9.81 * Vector3<double>::UnitZ());

 // Now the model is complete.
 mbw.Finalize();

 // We can now connect to DrakeVisualizer.
 mbw.ConnectDrakeVisualizer(&builder);
 @endcode
 */
template <class T>
class MultibodyWorld {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyWorld)

  explicit MultibodyWorld(
      
      const MultibodyPlant* plant, const SceneGraph* graph);



  /// Gets a constant reference to the MultibodyPlant. This function can be
  /// called pre-Finalize.
  const multibody::multibody_plant::MultibodyPlant<T>& plant() const {
    return *plant_;
  }

  /// Gets a mutable reference to the MultibodyPlant. This function can be
  /// called pre-Finalize.
  multibody::multibody_plant::MultibodyPlant<T>& mutable_plant() {
    return *plant_;
  }

  /// Gets a constant reference to the SceneGraph. This function can be
  /// called pre-Finalize.
  const geometry::SceneGraph<T>& scene_graph() const { return *scene_graph_; }

  /// Gets a mutable reference to the SceneGraph. This function can be
  /// called pre-Finalize.
  geometry::SceneGraph<T>& mutable_scene_graph() { return *scene_graph_; }

  /// Users *must* call Finalize() after making any additions to the
  /// MultibodyPlant and before using this class in the Systems framework.
  /// This should be called exactly once.
  ///
  /// @see multibody::multibody_plant::MultibodyPlant<T>::Finalize()
  void Finalize();

  /// Determines whether this system has been finalized (via a call to
  /// Finalize()).
  bool is_finalized() const {
    return plant_->is_finalized();
  }

 private:
  // The builder that builds this Diagram.
  std::unique_ptr<systems::DiagramBuilder<T>> builder_;

  // The pointer to the MultibodyPlant created by `builder_`
  multibody::multibody_plant::MultibodyPlant<T>* plant_{nullptr};

  // The pointer to the SceneGraph created by `builder_`.
  geometry::SceneGraph<T>* scene_graph_{nullptr};
};

template <class T>
MultibodyWorld<T>::MultibodyWorld(double time_step) {
  builder_ = std::make_unique<systems::DiagramBuilder<T>>();

  scene_graph_ = builder_->template AddSystem<geometry::SceneGraph<T>>();
  scene_graph_->set_name("scene_graph");

  // Create the necessary connections.
  

  plant_ =
      builder_->template AddSystem<
          multibody::multibody_plant::MultibodyPlant<T>>(time_step);
}

template <class T>
void MultibodyWorld<T>::Finalize() {
  // Verify that the system is not already finalized.
  if (is_finalized())
    throw std::logic_error("MultibodyWorld::Finalize() has already"
                               " been called");

  // MultibodyPlant must be finalized first.
  plant_->Finalize(scene_graph_);

  // Indicate that finalization is complete.
  finalized_ = true;
}

void Connect(
    MultibodyPlant<T>* plant, SceneGraph<T>* scene_graph,
    DiagramBuilder<T>* builder) {
  builder->Connect(
      plant->get_geometry_poses_output_port(),
      scene_graph->get_source_pose_port(plant->get_source_id().value()));
  builder->Connect(
      scene_graph->get_query_output_port(),
      plant->get_geometry_query_input_port());
}

}  // namespace drake

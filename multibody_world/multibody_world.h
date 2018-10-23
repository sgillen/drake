/** @file
 Provides functions to create a Diagram from a MultibodyPlant and a SceneGraph,
 allowing this diagram to be simulated (in time) and geometric queries to be
 posed, e.g., for planning applications.
*/

#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

/**
ISSUE:

Would like to maintain separate APIs for MBP+SG, to preserve separate APIs, as
well as

  MBP x N --> SG x 1 ???

However, to do meaningful queries, we need (MBP, SG, Diagram), so that we can
allocate contexts and extract what we want.

If we want IK to work with any MBP + SG, then we have to either transfer
ownership, or also pass in the diagram... in which case, we may have multiple
MBPs...

If we have that, we can instead provide:
* A 'model' Context to clone
* Some way to extract the relevant bits, i.e. `ResolveContext`
Such that we fix portions of MBPs... but meh.

Comes down to:
  Is there any use in having `diagram` be open-ended?
  - If not, we should probably just do MBW?
      This may lead to the APIs becoming even more coupled, and we may end up
      merging the two...
      Also, most APIs wanting the two will end up using whatever tuple is the
      most expressive... So it would end up being MBW...

      The question then is, how cheap is it construct this tuple?
        If they own it, super expensive... But they *have* to own, 'cause
        Systems.
        If they *didn't* have to own it, then all of this would be moot...

  - If so, then ... ???



Typeup:

My attempt at summarizing constraints and potential solutions:

AFAIK, the constraints for using MBP+SG boil down to:
* Having the carefully crafted three-tuple (MBP, SG, Diagram), where Diagram
  owns MBP+SG and has them artfully + properly connected.
* To allocate a context, a Diagram is *always* necessary, b/c doing
  `plant.get_geometry_input_port().Eval(...)` requires a neighboring SG context.
  * This comes from the fact that our (MBP, SG) interface is only usable via
  the Systems framework, and a Diagram must own its constituent systems.
  * We could remove the need for Diagram if we have a model `Context`, and
  enable Systems to extract their relevant bits (it'd be hacky, though).

If we go down that route:
* My estimate is that people will naturally gravitate to MBW, and all APIs
  will ultimately consume MBW rather than super special 3-tuple, because why
  should they, esp. if:
    * They don't need (AnySystem[n], ..., SG) interface
    * If rigid collisions via simulation only work with (MBP x 1, SG)...
* We will either have to (a) overload everything between super tuple and MBW,
  or (b) just use MBW.
    * If we do (b), my fear is we *may* run into the same issue as MBP vs. MBT:
    if users really only have one interface they care about

Potential options for MBW:
  * owning super tuple, like Evan's current MBW implementation:
    * We either have to forward all the ports, b/c the Systems framework cannot
      make a non-encapsulated diagram / aliased ports. (API coupling makes me
      sad...)
    * Constrains to (MBP x 1, SG) API.
  * optionally owning super tuple:
    * Permits adding other systems in; *possibly* stays open to
      (AnySystem[n], ..., SG)
    * Have to ensure we give some methods to craft the bits (call
      `MBP::RegisterAsSceneGraphSource`, `DB::Connect()`, `DB::Build()`, etc. at
      the correct time
    * Yucky semantics when handling owning vs. non-owning case.
  * a non-owning super tuple:
    * Easily preserves existing (MBP x N, SG) API.
    * Annoying extra item in the tuple when you want to construct and/or
      transfer ownership.
    * Simple-ish.


Looking more into `Context<>` bits:
* To get the relevant `plant` context, you always need to know about the diagram.
* For geometric queries, you can have `plant` evaluate on its sub-context, but that seems to implicitly pull from the `scene_graph` context.
* If you ever want to duplicate contexts (for multithreading), you can simply `Clone()` the local context, but 
*/

template <typename T>
void AddPlantForSceneGraph(
    std::unique_ptr<MultibodyPlant<T>> owned_plant,
    SceneGraph<T>* scene_graph,
    DiagramBuilder<T>* builder) {
  auto* plant = builder.AddSystem(std::move(owned_plant));
  if (!plant->get_source_id()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }
  builder->Connect(
      plant->get_geometry_poses_output_port(),
      scene_graph->get_source_pose_port(plant->get_source_id().value()));
  builder->Connect(
      scene_graph->get_query_output_port(),
      plant->get_geometry_query_input_port());
}

/**
 Wraps a MultibodyPlant and SceneGraph for ease of use outside of the Systems
 framework.
 @code
 systems::DiagramBuilder<double> builder;

 MultibodyWorld<double> mbw();
 auto& plant = mbw.mutable_multibody_plant();

 // Make and add the cart_pole model.
 AddModelFromSdfFile(filename, &plant, &mbw.mutable_scene_graph());

 // Now the model is complete.
 mbw.FinalizeAndBuild();
 @endcode
 */
template <class T>
class MultibodyWorld {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyWorld)

  explicit MultibodyWorld(double period = 0.0)
      : owned_plant_(new MultibodyPlant<T>(period)),
        plant_(owned_plant_.get()),
        owned_scene_graph_(new geometry::SceneGraph<T>>()),
        scene_graph_(owned_scene_graph_.get()) {
    plant_->RegisterAsSourceForSceneGraph(scene_graph_);
  }

  void FinalizeAndBuild() {
    plant_->Finalize(scene_graph_);
    DiagramBuilder<T> builder;
    builder.AddSystem(std::move(owned_scene_graph_));
    AddPlantForSceneGraph(std::move(owned_plant_), scene_graph_, &builder);
    diagram_ = builder.Build();
  }

  std::pair<unique_ptr<Context<T>>, Context<T>*> CreateDefaultContext() const {
    DRAKE_DEMAND(diagram_);
    return diagram_->CreateDefaultContext();
  }

  // :( YUCK
  Context<T>& GetMutablePlantContext(Context<T>* diagram_context) {
    DRAKE_DEMAND(diagram_);
    return diagram_->GetMutableSubsystemContext(*plant_, diagram_context);
  }

  // :( YUCK
  Context<T>& GetMutableSceneGraphContext(Context<T>* diagram_context) {
    DRAKE_DEMAND(diagram_);
    return diagram_->GetMutableSubsystemContext(*scene_graph_, diagram_context);
  }

  const MultibodyPlant<T>& plant() const {
    return *plant_;
  }

  MultibodyPlant<T>& mutable_plant() {
    DRAKE_DEMAND(!diagram_);
    return *plant_;
  }

  const geometry::SceneGraph<T>& scene_graph() const { return *scene_graph_; }

  geometry::SceneGraph<T>& mutable_scene_graph() {
    DRAKE_DEMAND(!diagram_);
    return *scene_graph_;
  }

 private:
  MultibodyPlant<T>* plant_{nullptr};
  geometry::SceneGraph<T>* scene_graph_{nullptr};
  std::unique_ptr<Diagram<T>> diagram_;
};

}  // namespace drake

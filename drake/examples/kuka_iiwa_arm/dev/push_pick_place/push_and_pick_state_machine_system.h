#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/kuka_iiwa_arm/dev/push_pick_place/push_and_pick_state_machine.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/action.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/world_state.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/systems/framework/system_symbolic_inspector.h"
#include "drake/examples/kuka_iiwa_arm/dev/push_pick_place/perception_base.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace push_and_pick {

/**
 * A class that implements the Finite-State-Machine logic for the
 * Push-Pick-Place demo.
 */
class PushAndPickStateMachineSystem : public systems::LeafSystem<double> {
 public:
  /**
   * Constructor for the PushAndPickStateMachineSystem
   * @param iiwa_base, The pose of the base of the IIWA robot system.
   * @param period_sec : The update interval of the unrestricted update of
   * this system. This should be bigger than that of the PlanSource components.
   */
  PushAndPickStateMachineSystem(
      const std::string& iiwa_model_path,
      const std::string& end_effector_name,
      const Isometry3<double>& iiwa_base,
      const double period_sec = 0.01,
      std::unique_ptr<PerceptionBase> perception = nullptr);

  std::unique_ptr<systems::AbstractValues> AllocateAbstractState()
  const override;

  // This kind of a system is not a direct feedthrough.
  optional<bool> DoHasDirectFeedthrough(int, int) const final {
    return false;
  }

  void SetDefaultState(const systems::Context<double>& context,
                       systems::State<double>* state) const override;

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
                                systems::State<double>* state) const override;

  /**
   * Getter for the input port corresponding to the abstract input with iiwa
   * state message (LCM `robot_state_t` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_iiwa_state()
  const {
    return this->get_input_port(input_port_iiwa_state_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with box
   * state message (LCM `botcore::robot_state_t` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_box_state() const {
    return this->get_input_port(input_port_box_state_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with the wsg
   * status message (LCM `lcmt_schunk_wsg_status` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_wsg_status()
  const {
    return this->get_input_port(input_port_wsg_status_);
  }

  const systems::InputPortDescriptor<double>& get_input_port_depth_image()
  const {
    return this->get_input_port(input_port_depth_image_);
  }
  const systems::InputPortDescriptor<double>& get_input_port_depth_frame()
  const {
    return this->get_input_port(input_port_depth_frame_);
  }
  const systems::InputPortDescriptor<double>&
  get_input_port_camera_update_time() const {
    return this->get_input_port(input_port_camera_update_time_);
  }


  const systems::OutputPort<double>& get_output_port_iiwa_plan()
  const {
    return this->get_output_port(output_port_iiwa_plan_);
  }

  const systems::OutputPort<double>& get_output_port_wsg_command()
  const {
    return this->get_output_port(output_port_wsg_command_);
  }

  const systems::OutputPort<double>& get_output_port_camera_needed()
  const {
    return this->get_output_port(output_port_camera_needed_);
  }

  /// Return the state of the pick and place state machine.
  PushAndPickState state(
      const systems::Context<double>&) const;

  /// Return the state of the pick and place world.  Note that this
  /// reference is into data contained inside the passed in context.
  const pick_and_place::WorldState& world_state(
      const systems::Context<double>&) const;

 private:
  void CalcIiwaPlan(
      const systems::Context<double>& context,
      robotlocomotion::robot_plan_t* iiwa_plan) const;

  void CalcWsgCommand(
      const systems::Context<double>& context,
      lcmt_schunk_wsg_command* wsg_command) const;

  void CalcCameraNeeded(
      const systems::Context<double>& context,
      bool* camera_needed) const;

  struct InternalState;

  RigidBodyTree<double> iiwa_tree_{};
  // Input ports.
  int input_port_iiwa_state_{-1};
  int input_port_box_state_{-1};
  int input_port_wsg_status_{-1};
  // - Camera.
  int input_port_depth_image_{-1};
  int input_port_depth_frame_{-1};
  int input_port_camera_update_time_{-1};
  // Output ports.
  int output_port_iiwa_plan_{-1};
  int output_port_wsg_command_{-1};
  int output_port_camera_needed_{-1};

  std::string iiwa_model_path_;
  std::string end_effector_name_;
  const Isometry3<double> iiwa_base_;

  const std::unique_ptr<
      manipulation::planner::ConstraintRelaxingIk> planner_{nullptr};

  std::unique_ptr<PerceptionBase> perception_;
};


}  // namespace push_and_pick
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

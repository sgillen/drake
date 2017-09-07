#include "drake/examples/kuka_iiwa_arm/dev/push_pick_place/push_and_pick_state_machine_system.h"

#include <utility>
#include <vector>

#include "bot_core/robot_state_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/examples/kuka_iiwa_arm/dev/push_pick_place/push_and_pick_state_machine.h"
#include "drake/systems/rendering/pose_vector.h"

using bot_core::robot_state_t;

namespace drake {

using systems::rendering::PoseVector;

namespace examples {
namespace kuka_iiwa_arm {
namespace {
/* index of iiwastate */
const int kStateIndex = 0;

robotlocomotion::robot_plan_t MakeDefaultIiwaPlan() {
  robotlocomotion::robot_plan_t default_plan{};
  default_plan.utime = 0;
  default_plan.num_states = 0;
  return default_plan;
}

lcmt_schunk_wsg_command MakeDefaultWsgCommand() {
  lcmt_schunk_wsg_command default_command{};
  default_command.utime = 0;
  default_command.target_position_mm = 110;  // maximum aperture
  default_command.force = 0;
  return default_command;
}
}  // namespace

using manipulation::planner::ConstraintRelaxingIk;

namespace push_and_pick {

struct PushAndPickStateMachineSystem::InternalState {
  InternalState(const std::string& iiwa_model_path,
                const std::string& end_effector_name)
      : world_state(iiwa_model_path, end_effector_name),
        state_machine(false),
        last_iiwa_plan(MakeDefaultIiwaPlan()),
        last_wsg_command(MakeDefaultWsgCommand()) {}

  ~InternalState() {}

  pick_and_place::WorldState world_state;
  PushAndPickStateMachine state_machine;
  robotlocomotion::robot_plan_t last_iiwa_plan;
  lcmt_schunk_wsg_command last_wsg_command;
};

PushAndPickStateMachineSystem::PushAndPickStateMachineSystem(
    const std::string& iiwa_model_path,
    const std::string& end_effector_name,
    const Isometry3<double>& iiwa_base,
    const double period_sec)
    : iiwa_model_path_(iiwa_model_path),
      end_effector_name_(end_effector_name),
      iiwa_base_(iiwa_base),
      planner_(std::make_unique<ConstraintRelaxingIk>(
          iiwa_model_path_, end_effector_name_, iiwa_base_)) {
  input_port_iiwa_state_ = this->DeclareAbstractInputPort().get_index();
  input_port_box_state_ = this->DeclareAbstractInputPort().get_index();
  input_port_wsg_status_ = this->DeclareAbstractInputPort().get_index();

  input_port_depth_image_ = this->DeclareAbstractInputPort().get_index();
  input_port_depth_frame_ =
      this->DeclareInputPort(
          kVectorValued, PoseVector<double>::kSize).get_index();

  output_port_iiwa_plan_ =
      this->DeclareAbstractOutputPort(
              MakeDefaultIiwaPlan(),
              &PushAndPickStateMachineSystem::CalcIiwaPlan)
          .get_index();

  output_port_wsg_command_ =
      this->DeclareAbstractOutputPort(
              MakeDefaultWsgCommand(),
              &PushAndPickStateMachineSystem::CalcWsgCommand)
          .get_index();

  this->DeclarePeriodicUnrestrictedUpdate(period_sec, 0);
}

std::unique_ptr<systems::AbstractValues>
PushAndPickStateMachineSystem::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals;
  abstract_vals.push_back(systems::AbstractValue::Make(
          InternalState(iiwa_model_path_, end_effector_name_)));
  return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
}

void PushAndPickStateMachineSystem::SetDefaultState(
    const systems::Context<double>&,
    systems::State<double>* state) const {
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(kStateIndex);
  internal_state = InternalState(iiwa_model_path_, end_effector_name_);
}

void PushAndPickStateMachineSystem::CalcIiwaPlan(
    const systems::Context<double>& context,
    robotlocomotion::robot_plan_t* iiwa_plan) const {
  /* Call actions based on state machine logic */
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  *iiwa_plan = internal_state.last_iiwa_plan;
}


void PushAndPickStateMachineSystem::CalcWsgCommand(
    const systems::Context<double>& context,
    lcmt_schunk_wsg_command* wsg_command) const {
  /* Call actions based on state machine logic */
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  *wsg_command = internal_state.last_wsg_command;
}

void PushAndPickStateMachineSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  // Extract Internal state.
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(kStateIndex);

  /* Update world state from inputs. */
  const robot_state_t& iiwa_state =
      this->EvalAbstractInput(context, input_port_iiwa_state_)
          ->GetValue<robot_state_t>();
  const robot_state_t& box_state =
      this->EvalAbstractInput(context, input_port_box_state_)
          ->GetValue<robot_state_t>();
  const ImageDepth32F& depth_image =
      this->EvalAbstractInput(context, input_port_depth_image_)
          ->GetValue<ImageDepth32F>();
  const PoseVector<double>& depth_frame =
      this->EvalVectorInput<PoseVector>(context, input_port_depth_frame_);

  internal_state.world_state.HandleIiwaStatus(iiwa_state);
  internal_state.world_state.HandleWsgStatus(wsg_status);
  internal_state.world_state.HandleObjectStatus(box_state);

  // TODO(eric.cousineau): Embed frame name into camera, and use timestamp from
  // LCM.
  internal_state.state_machine.ReadImage(
      internal_state.world_state.get_iiwa_time(),
      depth_image,
      depth_frame);

  PushAndPickStateMachine::IiwaPublishCallback iiwa_callback =
      ([&](const robotlocomotion::robot_plan_t* plan) {
        internal_state.last_iiwa_plan = *plan;
      });

  PushAndPickStateMachine::WsgPublishCallback wsg_callback =
      ([&](const lcmt_schunk_wsg_command* msg) {
        internal_state.last_wsg_command = *msg;
      });
  internal_state.state_machine.Update(
      internal_state.world_state, iiwa_callback, wsg_callback, planner_.get());
}

PushAndPickState PushAndPickStateMachineSystem::state(
    const systems::Context<double>& context) const {
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  return internal_state.state_machine.state();
}

const pick_and_place::WorldState& PushAndPickStateMachineSystem::world_state(
    const systems::Context<double>& context) const {
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  return internal_state.world_state;
}

}  // namespace push_and_pick
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

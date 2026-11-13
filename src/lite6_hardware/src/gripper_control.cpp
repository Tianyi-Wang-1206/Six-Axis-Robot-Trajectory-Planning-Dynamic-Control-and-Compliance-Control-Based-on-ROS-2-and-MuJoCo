#include "lite6_hardware/gripper_control.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace lite6_hardware
{
namespace {
int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
constexpr int64_t kHeartbeatNs = 250000000;
constexpr int64_t kPhysicsFreshNs = 100000000;
}

void GripperControl::initialize(mjModel * model, mjData * data)
{
  model_ = model; data_ = data;
  for (size_t i = 0; i < 2; ++i) {
    const std::string side = i == 0 ? "left" : "right";
    const auto name = "gripper_" + side + "_finger";
    const int joint = mj_name2id(model_, mjOBJ_JOINT, name.c_str());
    brakes_[i] = mj_name2id(model_, mjOBJ_EQUALITY, ("gripper_" + side + "_brake").c_str());
    bodies_[i] = mj_name2id(model_, mjOBJ_BODY, name.c_str());
    if (joint < 0 || brakes_[i] < 0 || bodies_[i] < 0) {
      throw std::runtime_error("Missing wide gripper joint/body/brake");
    }
    qpos_[i] = model_->jnt_qposadr[joint];
    dof_[i] = model_->jnt_dofadr[joint];
    data_->qpos[qpos_[i]] = i == 0 ? -0.0081 : 0.0081;
  }
  motor_ = mj_name2id(model_, mjOBJ_ACTUATOR, "gripper");
  if (motor_ < 0) {throw std::runtime_error("Missing gripper motor");}
  state_ = Snapshot{};
  // Force initialization of both brake references.
  state_.brake = false;
  brake();
}

void GripperControl::activate()
{
  channel_->commands.reset();
  channel_->ready_until.store(0);
  channel_->epoch.fetch_add(1);
  channel_->active.store(true);
  channel_->physics_stamp.store(0);
  state_.state = State::BLOCKED;
  brake();
}

void GripperControl::deactivate()
{
  channel_->active.store(false);
  channel_->ready_until.store(0);
  channel_->epoch.fetch_add(1);
}

void GripperControl::configure(const rclcpp::Node::SharedPtr & node)
{
  if (timer_ || !node) {return;}
  const auto channel = channel_;
  using Supervisor = lite6_interfaces::msg::SupervisorState;
  // Volatile subscription intentionally refuses a retained pre-restart Ready.
  supervisor_ = node->create_subscription<Supervisor>("/supervisor/state", rclcpp::QoS(1),
    [channel](const Supervisor & message) {
      bool ready = message.mode == Supervisor::MODE_NORMAL &&
        message.motion_phase == Supervisor::MOTION_IDLE && message.motion_allowed &&
        message.controller_fresh && message.joint_states_fresh &&
        !message.estop_latched && !message.waiting_for_settle;
      if (!ready) {channel->saw_nonready = true;}
      if (channel->estop_inhibit && ready && channel->saw_nonready) {
        channel->estop_inhibit = false;
      }
      ready = ready && !channel->estop_inhibit;
      if (!ready) {
        channel->ready_until.store(0);
        // Even a brief non-Ready interval invalidates all pending motion.
        channel->epoch.fetch_add(1);
      } else {
        const auto now = steady_ns();
        if (channel->ready_until.load() < now) {channel->epoch.fetch_add(1);}
        channel->ready_until.store(now + kHeartbeatNs);
      }
    });
  estop_ = node->create_subscription<std_msgs::msg::Int32>(
    "/ctc_controller/system_cmd", rclcpp::QoS(10), [channel](const std_msgs::msg::Int32 & msg) {
      if (msg.data != 1) {return;}
      channel->estop_inhibit = true;
      channel->saw_nonready = false;
      channel->ready_until.store(0);
      channel->epoch.fetch_add(1);
    });
  service_ = node->create_service<Service>("/gripper/command",
    [channel](const std::shared_ptr<Service::Request> request,
              std::shared_ptr<Service::Response> response) {
      const auto now = steady_ns();
      if (!channel->active.load() || channel->ready_until.load() <= now ||
          now - channel->physics_stamp.load() > kPhysicsFreshNs) {
        response->detail = "Gripper requires fresh robot Ready and active physics";
        return;
      }
      if (request->command != Service::Request::OPEN &&
          request->command != Service::Request::CLOSE &&
          request->command != Service::Request::CONFIGURE) {
        response->detail = "Only OPEN, CLOSE and CONFIGURE are supported";
        return;
      }
      Command command;
      command.action = request->command;
      command.issued_ns = now;
      command.sequence = ++channel->sequence;
      command.epoch = channel->epoch.load();
      if (command.action == Service::Request::CONFIGURE) {
        if (!std::isfinite(request->stiffness) || request->stiffness < 100 || request->stiffness > 20000 ||
            !std::isfinite(request->damping) || request->damping < 1 || request->damping > 200 ||
            !std::isfinite(request->max_force) || request->max_force < 0.5 || request->max_force > 10) {
          response->detail = "Require K=100..20000 N/m, D=1..200 N*s/m, force=0.5..10 N";
          return;
        }
        command.stiffness = request->stiffness;
        command.damping = request->damping;
        command.max_force = request->max_force;
      }
      if (!channel->commands.push(command)) {
        response->detail = "Gripper command queue is full";
        return;
      }
      response->accepted = true;
      response->sequence = command.sequence;
      response->detail = "Queued; observe /gripper/state for physical outcome";
    });
  auto publisher = node->create_publisher<State>("/gripper/state", rclcpp::QoS(1).reliable());
  const auto clock = node->get_clock();
  timer_ = node->create_wall_timer(std::chrono::milliseconds(20), [channel, publisher, clock] {
    const auto state = channel->snapshots.read();
    const auto now = steady_ns();
    State message;
    message.stamp = clock->now();
    message.active = channel->active.load() && now - channel->physics_stamp.load() < kPhysicsFreshNs;
    message.ready = message.active && state.ready && channel->ready_until.load() > now;
    message.state = message.ready ? state.state : State::BLOCKED;
    message.brake_engaged = state.brake;
    message.applied_sequence = state.sequence;
    message.position = state.feedback.position;
    message.velocity = state.feedback.velocity;
    message.effort = state.effort;
    message.stiffness = state.stiffness;
    message.damping = state.damping;
    message.max_force = state.max_force;
    switch (message.state) {
      case State::BLOCKED: message.detail = "Robot not Ready or feedback stale; opening/closing inhibited"; break;
      case State::HOLDING: message.detail = "Holding; issue a new OPEN or CLOSE command"; break;
      case State::OPENING: message.detail = "Opening"; break;
      case State::CLOSING: message.detail = "Closing"; break;
      case State::OPEN: message.detail = "Open"; break;
      case State::CLOSED: message.detail = "Closed"; break;
      case State::GRASPED: message.detail = "Contact grasp; force limited impedance hold"; break;
      default: message.detail = "Travel timed out; brake engaged. Check obstruction/gains, then retry"; break;
    }
    publisher->publish(message);
  });
}

GripperControl::Feedback GripperControl::feedback() const
{
  Feedback value;
  for (size_t i = 0; i < 2; ++i) {
    value.position[i] = data_->qpos[qpos_[i]];
    value.velocity[i] = data_->qvel[dof_[i]];
    value.acceleration[i] = data_->qacc[dof_[i]];
    value.effort[i] = data_->qfrc_actuator[dof_[i]];
  }
  return value;
}

bool GripperControl::finite() const
{
  const auto value = feedback();
  for (size_t i = 0; i < 2; ++i) {
    if (!std::isfinite(value.position[i]) || !std::isfinite(value.velocity[i]) ||
        !std::isfinite(value.acceleration[i]) || !std::isfinite(value.effort[i])) {return false;}
  }
  return true;
}

void GripperControl::brake()
{
  if (!state_.brake) {
    for (size_t i = 0; i < 2; ++i) {
      model_->eq_data[mjNEQDATA * brakes_[i]] = data_->qpos[qpos_[i]];
      data_->eq_active[brakes_[i]] = 1;
    }
  }
  state_.brake = true;
  data_->ctrl[motor_] = 0;
  state_.effort = 0;
  elapsed_ = settled_ = 0;
}

void GripperControl::publish(bool ready)
{
  state_.ready = ready;
  state_.feedback = feedback();
  channel_->snapshots.publish(state_);
}

void GripperControl::step(int64_t boundary_ns, int64_t now_ns)
{
  channel_->physics_stamp.store(now_ns);
  const auto epoch = channel_->epoch.load();
  const bool ready = channel_->active.load() && channel_->ready_until.load() > now_ns &&
    now_ns - boundary_ns < kPhysicsFreshNs;
  if (!ready || (!state_.brake && motion_epoch_ != epoch)) {
    brake();
    state_.state = State::BLOCKED;
  }
  if (ready && state_.state == State::BLOCKED) {state_.state = State::HOLDING;}
  Command command;
  while (channel_->commands.pop_due(boundary_ns, command)) {
    state_.sequence = command.sequence;
    if (!ready || command.epoch != epoch || now_ns - command.issued_ns > kHeartbeatNs) {continue;}
    if (command.action == Service::Request::CONFIGURE) {
      state_.stiffness = command.stiffness;
      state_.damping = command.damping;
      state_.max_force = command.max_force;
    } else {
      target_ = command.action == Service::Request::OPEN ? -0.0081 : -0.00001;
      state_.state = command.action == Service::Request::OPEN ? State::OPENING : State::CLOSING;
      state_.brake = false;
      motion_epoch_ = epoch;
      elapsed_ = settled_ = 0;
      for (int id : brakes_) {data_->eq_active[id] = 0;}
    }
  }
  if (state_.brake || !ready) {publish(ready); return;}
  const double q = data_->qpos[qpos_[0]], v = data_->qvel[dof_[0]];
  // The coupled right coordinate is -left: its generalized bias subtracts.
  const double feedforward = data_->qfrc_bias[dof_[0]] - data_->qfrc_bias[dof_[1]];
  state_.effort = std::clamp(state_.stiffness * (target_ - q) - state_.damping * v + feedforward,
    -state_.max_force, state_.max_force);
  data_->ctrl[motor_] = state_.effort;
  const double dt = model_->opt.timestep;
  elapsed_ += dt;
  const bool stationary = std::abs(v) < 0.001 && std::abs(data_->qvel[dof_[1]]) < 0.001;
  settled_ = stationary ? settled_ + dt : 0;
  if (settled_ > 0.15 && std::abs(target_ - q) < 0.00025 &&
      std::abs(-target_ - data_->qpos[qpos_[1]]) < 0.00025) {
    state_.state = target_ < -0.004 ? State::OPEN : State::CLOSED;
  } else if (target_ > -0.004 && settled_ > 0.25) {
    // Require a real external finger contact, not friction alone, for GRASPED.
    bool contact = false;
    for (int i = 0; i < data_->ncon; ++i) {
      const auto & c = data_->contact[i];
      if (c.efc_address < 0 || c.geom[0] < 0 || c.geom[1] < 0) {continue;}
      const int a = model_->geom_bodyid[c.geom[0]], b = model_->geom_bodyid[c.geom[1]];
      const bool fa = a == bodies_[0] || a == bodies_[1];
      const bool fb = b == bodies_[0] || b == bodies_[1];
      contact = contact || (fa != fb);
    }
    if (contact) {state_.state = State::GRASPED;}
  }
  if (elapsed_ > 8.0 && (state_.state == State::OPENING || state_.state == State::CLOSING)) {
    state_.state = State::FAULT;
    brake();
  }
  publish(ready);
}
}  // namespace lite6_hardware

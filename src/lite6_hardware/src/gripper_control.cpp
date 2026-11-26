#include "lite6_hardware/gripper_control.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace lite6_hardware
{
namespace {
int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
constexpr int64_t kCommandAgeNs = 250000000;
constexpr int64_t kPhysicsFreshNs = 100000000;
}

void GripperControl::initialize(mjModel * model, mjData * data)
{
  model_ = model; data_ = data;
  const auto config = YAML::LoadFile(ament_index_cpp::get_package_share_directory(
    "lite6_hardware") + "/config/gripper.yaml")["gripper"];
  auto positive = [&config](const char * key) {
    const double value = config[key].as<double>();
    if (!std::isfinite(value) || value <= 0) {
      throw std::runtime_error(std::string("Invalid positive gripper parameter: ") + key);
    }
    return value;
  };
  parameters_ = {positive("stiffness"), positive("damping"), positive("max_force"),
    positive("max_velocity"), positive("max_acceleration"), positive("position_tolerance"),
    positive("stationary_velocity"), positive("settle_time"), positive("travel_timeout"),
    positive("grasp_force"), positive("grasp_confirm_time"), positive("grasp_loss_time"),
    positive("contact_normal_cosine")};
  if (parameters_.max_velocity > 0.05 || parameters_.contact_normal_cosine > 1 ||
      parameters_.stationary_velocity >= parameters_.max_velocity ||
      parameters_.grasp_confirm_time >= parameters_.travel_timeout ||
      parameters_.settle_time >= parameters_.travel_timeout ||
      2 * parameters_.grasp_force >= parameters_.max_force) {
    throw std::runtime_error("Gripper parameters exceed travel/contact/velocity limits");
  }
  for (size_t i = 0; i < 2; ++i) {
    const std::string name = i == 0 ? "gripper_left_finger" : "gripper_right_finger";
    joints_[i] = mj_name2id(model_, mjOBJ_JOINT, name.c_str());
    tips_[i] = mj_name2id(model_, mjOBJ_GEOM, name.c_str());
    if (joints_[i] < 0 || tips_[i] < 0) {throw std::runtime_error("Missing gripper joint/tip");}
    qpos_[i] = model_->jnt_qposadr[joints_[i]];
    dof_[i] = model_->jnt_dofadr[joints_[i]];
  }
  open_ = model_->jnt_range[2*joints_[0]];
  closed_ = model_->jnt_range[2*joints_[0]+1];
  if (parameters_.position_tolerance >= (closed_ - open_) / 4) {
    throw std::runtime_error("Gripper position tolerance too large for stroke");
  }
  data_->qpos[qpos_[0]] = open_;
  data_->qpos[qpos_[1]] = -open_;
  motor_ = mj_name2id(model_, mjOBJ_ACTUATOR, "gripper");
  robot_root_ = mj_name2id(model_, mjOBJ_BODY, "link_base");
  if (motor_ < 0 || robot_root_ < 0 ||
      parameters_.max_force > model_->actuator_forcerange[2*motor_+1] ||
      -parameters_.max_force < model_->actuator_forcerange[2*motor_]) {
    throw std::runtime_error("Missing gripper motor/base or force exceeds motor rating");
  }
  contact_forces_.resize(model_->nbody);
  is_robot_body_.assign(static_cast<size_t>(model_->nbody), 0U);
  for (int b = 0; b < model_->nbody; ++b) {
    for (int parent = b; parent > 0; parent = model_->body_parentid[parent]) {
      if (parent == robot_root_) {
        is_robot_body_[static_cast<size_t>(b)] = 1U;
        break;
      }
    }
  }
  state_ = Snapshot{};
  const double total_stiction =
    std::max(0.0, model_->dof_frictionloss[dof_[0]]) +
    std::max(0.0, model_->dof_frictionloss[dof_[1]]);
  min_force_override_ = std::max(2.0 * parameters_.grasp_force, total_stiction + 2.0 * parameters_.grasp_force);
  if (min_force_override_ > parameters_.max_force) {
    throw std::runtime_error("Configured gripper max_force cannot overcome stiction and grasp threshold");
  }
  active_max_force_ = parameters_.max_force;
  state_.max_force = active_max_force_;
  stop_drive(State::STOPPED);
}

void GripperControl::activate()
{
  channel_->active.store(false, std::memory_order_seq_cst);
  channel_->epoch.fetch_add(1, std::memory_order_acq_rel);
  // Drain any in-flight service callbacks before resetting the SPSC queue.
  while (channel_->inflight_callbacks.load(std::memory_order_acquire) != 0U) {
    std::this_thread::yield();
  }
  channel_->commands.reset();
  channel_->physics_stamp.store(0);
  active_max_force_ = parameters_.max_force;
  state_.max_force = active_max_force_;
  state_.last_command = 0;
  state_.last_cmd_result = State::CMD_RES_NONE;
  stop_drive(State::STOPPED);
  channel_->active.store(true, std::memory_order_release);
}

void GripperControl::deactivate()
{
  channel_->active.store(false, std::memory_order_seq_cst);
  channel_->epoch.fetch_add(1, std::memory_order_acq_rel);
}

void GripperControl::configure(const rclcpp::Node::SharedPtr & node)
{
  if (timer_ || !node) {return;}
  const auto channel = channel_;
  const auto params = parameters_;
  const double min_override = min_force_override_;
  service_ = node->create_service<Service>("/gripper/command",
    [channel, params, min_override](const std::shared_ptr<Service::Request> request,
                      std::shared_ptr<Service::Response> response) {
      struct InflightGuard
      {
        std::atomic<uint32_t> & count;
        ~InflightGuard() { count.fetch_sub(1U, std::memory_order_release); }
      };
      channel->inflight_callbacks.fetch_add(1U, std::memory_order_acq_rel);
      InflightGuard guard{channel->inflight_callbacks};
      std::lock_guard<std::mutex> lock(channel->service_mutex);

      const auto now = steady_ns();
      if (!channel->active.load(std::memory_order_acquire) ||
          now - channel->physics_stamp.load() > kPhysicsFreshNs)
      {
        response->detail = "Gripper hardware inactive or physics feedback stale";
        return;
      }
      if (request->command != Service::Request::OPEN &&
          request->command != Service::Request::CLOSE && request->command != Service::Request::STOP) {
        response->detail = "Only OPEN, CLOSE and STOP are supported";
        return;
      }
      if (!std::isfinite(request->max_force_override) ||
          request->max_force_override < 0.0 ||
          request->max_force_override > params.max_force) {
        response->detail = "max_force_override must be finite and in [0.0, configured max_force]";
        return;
      }
      if (request->max_force_override > 0.0 && request->max_force_override < min_override) {
        response->detail =
          "Positive max_force_override is below minimum effective drive/grasp force (" +
          std::to_string(min_override) + " N); use 0.0 for default or >= " +
          std::to_string(min_override) + " N";
        return;
      }
      Command command;
      command.action = request->command;
      command.issued_ns = now;
      command.sequence = channel->sequence.fetch_add(1ULL, std::memory_order_acq_rel) + 1ULL;
      command.epoch = channel->epoch.load(std::memory_order_acquire);
      command.max_force_override = request->max_force_override;
      if (command.action == Service::Request::STOP) {
        channel->stops.publish(command);
      } else if (!channel->commands.push(command)) {
        response->detail = "Gripper motion queue is full; STOP remains available";
        return;
      }
      response->accepted = true;
      response->sequence = command.sequence;
      response->detail = "Queued; observe /gripper/state for physical outcome";
    });
  auto publisher = node->create_publisher<State>("/gripper/state", rclcpp::QoS(1).reliable());
  const auto clock = node->get_clock();
  timer_ = node->create_wall_timer(std::chrono::milliseconds(20), [channel, publisher, clock, params] {
    const auto state = channel->snapshots.read();
    State message;
    message.stamp = clock->now();
    message.active = channel->active.load() &&
      steady_ns() - channel->physics_stamp.load() < kPhysicsFreshNs;
    message.ready = message.active && state.ready;
    message.state = message.ready ? state.state : State::BLOCKED;
    message.applied_sequence = state.sequence;
    message.last_command = state.last_command;
    if (message.ready) {
      message.last_cmd_result = state.last_cmd_result;
    } else if (state.last_cmd_result == State::CMD_RES_IN_PROGRESS) {
      message.last_cmd_result = State::CMD_RES_BLOCKED;
    } else {
      message.last_cmd_result = state.last_cmd_result;
    }
    message.position = state.feedback.position;
    message.velocity = state.feedback.velocity;
    message.effort = state.effort;
    message.stiffness = params.stiffness;
    message.damping = params.damping;
    message.max_force = state.max_force > 0.0 ? state.max_force : params.max_force;
    message.max_velocity = params.max_velocity;
    switch (message.state) {
      case State::BLOCKED: message.detail = "Gripper hardware inactive or feedback stale"; break;
      case State::STOPPED: message.detail = "Stopped; motor drive off"; break;
      case State::OPENING: message.detail = "Opening"; break;
      case State::CLOSING: message.detail = "Closing; grasp not confirmed"; break;
      case State::OPEN: message.detail = "Open"; break;
      case State::CLOSED: message.detail = "Closed; no object grasp confirmed"; break;
      case State::GRASPED: message.detail = "Grasp estimated from sustained opposing fingertip forces"; break;
      default: message.detail = "Travel timed out; motor drive off. Check obstruction, then retry"; break;
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

void GripperControl::stop_drive(uint8_t state)
{
  action_ = Service::Request::STOP;
  state_.state = state;
  data_->ctrl[motor_] = state_.effort = 0;
  reference_ = (data_->qpos[qpos_[0]] - data_->qpos[qpos_[1]]) / 2;
  reference_velocity_ = elapsed_ = settled_ = grasp_time_ = lost_time_ = 0;
  candidate_object_ = grasped_object_ = -1;
}

int GripperControl::grasp_contact()
{
  std::fill(contact_forces_.begin(), contact_forces_.end(), std::array<double, 2>{});
  for (int k = 0; k < data_->ncon; ++k) {
    const auto & contact = data_->contact[k];
    if (contact.efc_address < 0 || contact.geom[0] < 0 || contact.geom[1] < 0) {continue;}
    for (size_t finger = 0; finger < 2; ++finger) {
      const int side = contact.geom[0] == tips_[finger] ? 0 :
        (contact.geom[1] == tips_[finger] ? 1 : -1);
      if (side < 0) {continue;}
      const int body = model_->geom_bodyid[contact.geom[1-side]];
      // Only an external movable rigid body qualifies; exclude table, world,
      // robot links and the other finger. Multiple geoms may belong to one body.
      const bool robot = body >= 0 &&
        static_cast<size_t>(body) < is_robot_body_.size() &&
        is_robot_body_[static_cast<size_t>(body)] != 0U;
      const int object = model_->body_weldid[body];
      if (robot || object <= 0 || model_->body_dofnum[object] == 0) {continue;}
      const auto * axis = data_->xaxis + 3*joints_[finger];
      const double inward = (side == 0 ? 1.0 : -1.0) * (finger == 0 ? 1.0 : -1.0) *
        mju_dot3(contact.frame, axis);
      if (inward < parameters_.contact_normal_cosine) {continue;}
      mjtNum force[6];
      mj_contactForce(model_, data_, k, force);
      contact_forces_[object][finger] += std::max(0.0, force[0]) * inward;
    }
  }
  for (size_t object = 1; object < contact_forces_.size(); ++object) {
    if (contact_forces_[object][0] >= parameters_.grasp_force &&
        contact_forces_[object][1] >= parameters_.grasp_force) {return static_cast<int>(object);}
  }
  return -1;
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
  const bool ready = channel_->active.load() && now_ns - boundary_ns < kPhysicsFreshNs;
  if (!ready) {
    stop_drive(State::BLOCKED);
    if (state_.last_cmd_result == State::CMD_RES_IN_PROGRESS) {
      state_.last_cmd_result = State::CMD_RES_BLOCKED;
    }
  } else if (state_.state == State::BLOCKED) {
    stop_drive(State::STOPPED);
  }
  const auto stop = channel_->stops.read();
  if (stop.sequence > state_.sequence && stop.epoch == epoch && stop.issued_ns <= boundary_ns) {
    stop_drive(ready ? State::STOPPED : State::BLOCKED);
    state_.sequence = stop.sequence;
    state_.last_command = Service::Request::STOP;
    state_.last_cmd_result = ready ? State::CMD_RES_STOPPED : State::CMD_RES_BLOCKED;
  }
  Command command;
  while (channel_->commands.pop_due(boundary_ns, command)) {
    if (command.epoch != epoch || command.sequence <= state_.sequence) {continue;}
    state_.sequence = command.sequence;
    state_.last_command = command.action;
    if (!ready) {
      stop_drive(State::BLOCKED);
      state_.last_cmd_result = State::CMD_RES_BLOCKED;
      continue;
    }
    if (now_ns - command.issued_ns > kCommandAgeNs) {
      stop_drive(State::FAULT);
      state_.last_cmd_result = State::CMD_RES_FAULT;
      continue;
    }
    stop_drive(State::STOPPED);
    active_max_force_ = command.max_force_override > 0.0 ?
      std::min(command.max_force_override, parameters_.max_force) : parameters_.max_force;
    state_.max_force = active_max_force_;
    action_ = command.action;
    target_ = action_ == Service::Request::OPEN ? open_ : closed_;
    state_.state = action_ == Service::Request::OPEN ? State::OPENING : State::CLOSING;
    state_.last_cmd_result = State::CMD_RES_IN_PROGRESS;
  }
  if (action_ == Service::Request::STOP || !ready) {publish(ready); return;}

  const auto & p = parameters_;
  const double max_force = active_max_force_ > 0.0 ? active_max_force_ : p.max_force;
  const double dt = model_->opt.timestep;
  const double left = data_->qpos[qpos_[0]], right = data_->qpos[qpos_[1]];
  const double vl = data_->qvel[dof_[0]], vr = data_->qvel[dof_[1]];
  const double q = (left - right) / 2, v = (vl - vr) / 2;
  const double distance = target_ - reference_;
  const double requested_velocity = std::copysign(
    std::min(p.max_velocity, std::sqrt(2*p.max_acceleration*std::abs(distance))), distance);
  reference_velocity_ += std::clamp(requested_velocity - reference_velocity_,
    -p.max_acceleration*dt, p.max_acceleration*dt);
  const double advance = reference_velocity_*dt;
  if (distance*advance >= 0 && std::abs(advance) >= std::abs(distance)) {
    reference_ = target_; reference_velocity_ = 0;
  } else {reference_ += advance;}
  // Bound position windup against an obstruction; retain finite closing force.
  const double max_error = max_force / p.stiffness;
  reference_ = std::clamp(reference_, q-max_error, q+max_error);
  const double feedforward = data_->qfrc_bias[dof_[0]] - data_->qfrc_bias[dof_[1]];
  const double effort = p.stiffness*(reference_-q) + p.damping*(reference_velocity_-v) + feedforward;
  // Finite torque-speed envelope: drive falls to zero at the rated finger
  // speed and becomes braking above it. Never clamp qpos/qvel or lock a jaw.
  // Use both measured jaw speeds so coupling error cannot hide an overspeed.
  const double upper = max_force*std::clamp(1-std::max(vl,-vr)/p.max_velocity, -1.0, 1.0);
  const double lower = -max_force*std::clamp(1+std::min(vl,-vr)/p.max_velocity, -1.0, 1.0);
  state_.effort = lower <= upper ? std::clamp(effort, lower, upper) :
    std::clamp(-p.damping*v, -max_force, max_force);
  data_->ctrl[motor_] = state_.effort;

  const bool stationary = std::abs(vl) < p.stationary_velocity && std::abs(vr) < p.stationary_velocity;
  settled_ = stationary ? settled_+dt : 0;
  const bool endpoint = std::abs(target_-left) < p.position_tolerance &&
    std::abs(-target_-right) < p.position_tolerance;
  const bool hold_band = std::abs(target_-left) < 2.0*p.position_tolerance &&
    std::abs(-target_-right) < 2.0*p.position_tolerance;
  const int object = action_ == Service::Request::CLOSE && !endpoint ? grasp_contact() : -1;
  if (object >= 0 && stationary) {
    grasp_time_ = object == candidate_object_ ? grasp_time_+dt : dt;
    candidate_object_ = object;
  } else {grasp_time_ = 0; candidate_object_ = -1;}

  if (state_.state == State::GRASPED) {
    lost_time_ = object == grasped_object_ && stationary ? 0 : lost_time_+dt;
    if (lost_time_ >= p.grasp_loss_time) {
      state_.state = State::CLOSING;
      state_.last_cmd_result = State::CMD_RES_IN_PROGRESS;
      grasped_object_ = -1;
      elapsed_ = 0;
    }
  }
  if (endpoint && settled_ >= p.settle_time) {
    state_.state = action_ == Service::Request::OPEN ? State::OPEN : State::CLOSED;
    state_.last_cmd_result = State::CMD_RES_SUCCEEDED;
    grasped_object_ = -1;
  } else if (grasp_time_ >= p.grasp_confirm_time) {
    state_.state = State::GRASPED;
    state_.last_cmd_result = State::CMD_RES_SUCCEEDED;
    grasped_object_ = object;
    lost_time_ = 0;
  } else if ((state_.state == State::OPEN || state_.state == State::CLOSED) && !hold_band) {
    state_.state = action_ == Service::Request::OPEN ? State::OPENING : State::CLOSING;
    state_.last_cmd_result = State::CMD_RES_IN_PROGRESS;
    elapsed_ = 0;
  }
  if (state_.state == State::OPENING || state_.state == State::CLOSING) {
    elapsed_ += dt;
    if (elapsed_ >= p.travel_timeout) {
      stop_drive(State::FAULT);
      state_.last_cmd_result = State::CMD_RES_FAULT;
    }
  } else {elapsed_ = 0;}
  publish(ready);
}
}  // namespace lite6_hardware

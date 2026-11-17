#ifndef LITE6_HARDWARE__GRIPPER_CONTROL_HPP_
#define LITE6_HARDWARE__GRIPPER_CONTROL_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <vector>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include "lite6_hardware/simulation_exchange.hpp"
#include "lite6_interfaces/msg/gripper_state.hpp"
#include "lite6_interfaces/srv/gripper_command.hpp"

namespace lite6_hardware
{
// Independent OPEN/CLOSE/STOP drive. All MuJoCo access is on the physics
// thread; ROS callbacks exchange commands/status without locks or allocation in step().
class GripperControl
{
public:
  using State = lite6_interfaces::msg::GripperState;
  using Service = lite6_interfaces::srv::GripperCommand;
  struct Feedback {
    std::array<double, 2> position{}, velocity{}, acceleration{}, effort{};
  };
  void initialize(mjModel * model, mjData * data);
  void configure(const rclcpp::Node::SharedPtr & node);
  void activate();
  void deactivate();
  void step(int64_t boundary_ns, int64_t now_ns);
  Feedback feedback() const;
  bool finite() const;

private:
  struct Parameters {
    double stiffness, damping, max_force, max_velocity, max_acceleration;
    double position_tolerance, stationary_velocity, settle_time, travel_timeout;
    double grasp_force, grasp_confirm_time, grasp_loss_time, contact_normal_cosine;
  } parameters_{};
  struct Command {
    int64_t issued_ns = 0;
    uint64_t sequence = 0, epoch = 0;
    uint8_t action = 0;
  };
  struct Snapshot {
    Feedback feedback;
    uint8_t state = State::BLOCKED;
    bool ready = false;
    uint64_t sequence = 0;
    double effort = 0.0;
  };
  struct Channel {
    detail::SpscQueue<Command, 32> commands;
    // STOP cannot be refused because the motion queue is full.
    detail::LatestSnapshot<Command> stops;
    detail::LatestSnapshot<Snapshot> snapshots;
    std::atomic<int64_t> physics_stamp{0};
    std::atomic<uint64_t> epoch{0};
    std::atomic<bool> active{false};
    std::atomic<uint32_t> inflight_callbacks{0};
    std::atomic<uint64_t> sequence{0};
  };
  void stop_drive(uint8_t state);
  int grasp_contact();
  void publish(bool ready);
  mjModel * model_ = nullptr;
  mjData * data_ = nullptr;
  std::array<int, 2> qpos_{}, dof_{}, joints_{}, tips_{};
  int robot_root_ = -1;
  std::vector<uint8_t> is_robot_body_;
  std::vector<std::array<double, 2>> contact_forces_;
  int motor_ = -1;
  Snapshot state_;
  double open_ = 0.0, closed_ = 0.0, target_ = 0.0, reference_ = 0.0, reference_velocity_ = 0.0;
  double elapsed_ = 0.0, settled_ = 0.0, grasp_time_ = 0.0, lost_time_ = 0.0;
  int candidate_object_ = -1, grasped_object_ = -1;
  uint8_t action_ = Service::Request::STOP;
  std::shared_ptr<Channel> channel_ = std::make_shared<Channel>();
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace lite6_hardware
#endif

#ifndef LITE6_HARDWARE__GRIPPER_CONTROL_HPP_
#define LITE6_HARDWARE__GRIPPER_CONTROL_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include "lite6_hardware/simulation_exchange.hpp"
#include "lite6_interfaces/msg/gripper_state.hpp"
#include "lite6_interfaces/msg/supervisor_state.hpp"
#include "lite6_interfaces/srv/gripper_command.hpp"

namespace lite6_hardware
{
// Independent binary gripper controller. All MuJoCo access is on the physics
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
  struct Command {
    int64_t issued_ns = 0;
    uint64_t sequence = 0, epoch = 0;
    uint8_t action = 0;
    double stiffness = 4000.0, damping = 40.0, max_force = 10.0;
  };
  struct Snapshot {
    Feedback feedback;
    uint8_t state = State::BLOCKED;
    bool brake = true, ready = false;
    uint64_t sequence = 0;
    double stiffness = 4000.0, damping = 40.0, max_force = 10.0, effort = 0.0;
  };
  struct Channel {
    detail::SpscQueue<Command, 32> commands;
    detail::LatestSnapshot<Snapshot> snapshots;
    std::atomic<int64_t> ready_until{0}, physics_stamp{0};
    std::atomic<uint64_t> epoch{0};
    std::atomic<bool> active{false};
    // Only mutually exclusive ROS callbacks access these fields.
    uint64_t sequence = 0;
    bool estop_inhibit = false, saw_nonready = false;
  };
  void brake();
  void publish(bool ready);
  mjModel * model_ = nullptr;
  mjData * data_ = nullptr;
  std::array<int, 2> qpos_{}, dof_{}, brakes_{}, bodies_{};
  int motor_ = -1;
  Snapshot state_;
  double target_ = -0.0081, elapsed_ = 0.0, settled_ = 0.0;
  uint64_t motion_epoch_ = 0;
  std::shared_ptr<Channel> channel_ = std::make_shared<Channel>();
  rclcpp::Subscription<lite6_interfaces::msg::SupervisorState>::SharedPtr supervisor_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr estop_;
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace lite6_hardware
#endif

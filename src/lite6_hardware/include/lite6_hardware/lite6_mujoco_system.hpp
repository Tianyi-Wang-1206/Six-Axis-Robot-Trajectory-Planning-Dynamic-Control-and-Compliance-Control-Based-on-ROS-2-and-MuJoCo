#ifndef LITE6_HARDWARE__LITE6_MUJOCO_SYSTEM_HPP_
#define LITE6_HARDWARE__LITE6_MUJOCO_SYSTEM_HPP_

#include <vector>
#include <string>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lite6_hardware/lugre_friction.hpp"
#include "lite6_hardware/simulation_exchange.hpp"

#include <mujoco/mujoco.h>

namespace lite6_hardware
{

class Lite6MujocoSystem : public hardware_interface::SystemInterface
{
public:
  ~Lite6MujocoSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using Clock = std::chrono::steady_clock;
  static constexpr size_t kJoints = 6;
  static constexpr int64_t kStepNs = 100000;  // Fixed 0.1 ms physics step.
  struct EffortCommand
  {
    std::array<double, kJoints> effort{};
    int64_t issued_ns = 0;
    uint64_t sequence = 0;
  };
  struct StateSnapshot
  {
    std::array<double, kJoints> position{}, velocity{}, effort{};
    double simulation_time = 0.0;
    double max_lag = 0.0;
    int64_t origin_ns = 0, boundary_ns = 0, published_ns = 0, command_ns = 0;
    uint64_t steps = 0, command_sequence = 0;
    bool started = false;
  };
  enum class Fault {NONE, COMMAND_OVERFLOW, INVALID_COMMAND, INVALID_STATE, PHYSICS_EXCEPTION};
  struct Diagnostics
  {
    // Separate consumer from read(), so publishing never races state acquisition.
    detail::LatestSnapshot<StateSnapshot> snapshots;
    std::atomic<bool> active{false};
    std::atomic<Fault> fault{Fault::NONE};
    std::atomic<int64_t> last_write_ns{0};
    std::atomic<int64_t> max_write_interval_ns{0};
  };
  static int64_t steady_now_ns();
  void physics_loop();
  bool advance_physics(int64_t boundary_ns, EffortCommand & applied);
  void publish_snapshot(StateSnapshot & snapshot);
  void stop_physics();
  void release_model();
  void configure_diagnostics();

  // mjData and LuGre state belong exclusively to physics_thread_ while active.
  mjModel * m_model = nullptr;
  mjData * m_data = nullptr;
  std::thread physics_thread_;
  std::atomic<bool> stop_requested_{true};
  int physics_thread_priority_ = 60;
  uint64_t command_sequence_ = 0;  // write() only, reset on activation.
  detail::SpscQueue<EffortCommand, 4096> commands_;
  detail::LatestSnapshot<StateSnapshot> states_;
  std::shared_ptr<Diagnostics> diagnostics_ = std::make_shared<Diagnostics>();
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  std::vector<double> hw_commands_effort_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;

  std::vector<int> qpos_indices_;
  std::vector<int> qvel_indices_;
  std::vector<int> ctrl_indices_;
  std::vector<LuGreFriction> friction_models_;
};

}  // namespace lite6_hardware

#endif  // LITE6_HARDWARE__LITE6_MUJOCO_SYSTEM_HPP_

#ifndef LITE6_HARDWARE__LITE6_MUJOCO_SYSTEM_HPP_
#define LITE6_HARDWARE__LITE6_MUJOCO_SYSTEM_HPP_

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lite6_hardware/lugre_friction.hpp"

#include <mujoco/mujoco.h>

namespace lite6_hardware
{

class Lite6MujocoSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  mjModel * m_model = nullptr;
  mjData * m_data = nullptr;
  // Latch the first effective effort command; only write() advances this state.
  bool sim_started_ = false;

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

#ifndef PANDA_GAZEBO_HARDWARE__PANDA_GRAVITY_COMPENSATED_SYSTEM_HPP_
#define PANDA_GAZEBO_HARDWARE__PANDA_GRAVITY_COMPENSATED_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace panda_gazebo_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class PandaGravityCompensatedSystemPrivate;

class PandaGravityCompensatedSystem : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    std::map<std::string, sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    sim::EntityComponentManager & ecm,
    int & update_rate) override;

private:
  std::unique_ptr<PandaGravityCompensatedSystemPrivate> data_ptr_;
};

}  // namespace panda_gazebo_hardware

#endif  // PANDA_GAZEBO_HARDWARE__PANDA_GRAVITY_COMPENSATED_SYSTEM_HPP_

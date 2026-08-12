#include "panda_gazebo_hardware/panda_gravity_compensated_system.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef GZ_HEADERS
#include <gz/physics/Geometry.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#define GZ_PHYSICS_NAMESPACE gz::physics::
#define GZ_VECTOR_DOT dot
#else
#include <ignition/math/Vector3.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointTransmittedWrench.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityReset.hh>
#define GZ_PHYSICS_NAMESPACE ignition::math::
#define GZ_VECTOR_DOT Dot
#endif

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter_client.hpp>
#include <urdf/model.h>

namespace
{

using namespace std::chrono_literals;

struct JointData
{
  std::string name;
  sdf::JointType joint_type{sdf::JointType::INVALID};
  sdf::JointAxis joint_axis;
  double joint_position{0.0};
  double joint_position_cmd{0.0};
  double joint_velocity{0.0};
  double joint_effort{0.0};
  double joint_effort_cmd{0.0};
  sim::Entity sim_joint{sim::kNullEntity};
  std::string effort_interface_name;
  std::string position_interface_name;
  bool has_effort_command_interface{false};
  bool has_position_command_interface{false};
};

std::string getHardwareParameter(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & key,
  const std::string & default_value)
{
  const auto it = hardware_info.hardware_parameters.find(key);
  if (it == hardware_info.hardware_parameters.end()) {
    return default_value;
  }
  return it->second;
}

KDL::Vector parseGravityVector(const std::string & value)
{
  std::istringstream stream(value);
  double x = 0.0;
  double y = 0.0;
  double z = -9.81;
  if (!(stream >> x >> y >> z)) {
    throw std::runtime_error("Failed to parse gravity_vector hardware parameter: " + value);
  }
  return {x, y, z};
}

}  // namespace

namespace panda_gazebo_hardware
{

class PandaGravityCompensatedSystemPrivate
{
public:
  std::vector<JointData> joints;
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  sim::EntityComponentManager * ecm{nullptr};
  std::unique_ptr<KDL::Chain> chain;
  std::unique_ptr<KDL::ChainDynParam> gravity_solver;
  std::vector<int> joint_to_chain_index;
  std::string robot_description_node;
  std::string robot_description_param;
  std::string root_link;
  std::string tip_link;
  KDL::Vector gravity_vector{0.0, 0.0, -9.81};
  bool effort_command_active{true};
};

CallbackReturn PandaGravityCompensatedSystem::on_init(
  const hardware_interface::HardwareInfo & system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PandaGravityCompensatedSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto logger =
    this->nh_ ? this->nh_->get_logger() : rclcpp::get_logger("panda_gazebo_hardware");
  RCLCPP_INFO(logger, "Panda gravity-compensated Gazebo hardware configured");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PandaGravityCompensatedSystem::export_state_interfaces()
{
  return std::move(this->data_ptr_->state_interfaces);
}

std::vector<hardware_interface::CommandInterface>
PandaGravityCompensatedSystem::export_command_interfaces()
{
  return std::move(this->data_ptr_->command_interfaces);
}

CallbackReturn PandaGravityCompensatedSystem::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  for (auto & joint : this->data_ptr_->joints) {
    joint.joint_effort_cmd = 0.0;
    joint.joint_position_cmd = joint.joint_position;
  }
  return hardware_interface::SystemInterface::on_activate(previous_state);
}

CallbackReturn PandaGravityCompensatedSystem::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  return hardware_interface::SystemInterface::on_deactivate(previous_state);
}

hardware_interface::return_type PandaGravityCompensatedSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (const auto & interface_name : stop_interfaces) {
    const auto is_effort_interface = std::any_of(
      this->data_ptr_->joints.begin(),
      this->data_ptr_->joints.end(),
      [&interface_name](const JointData & joint) {
        return interface_name == joint.effort_interface_name;
      });
    if (is_effort_interface) {
      this->data_ptr_->effort_command_active = false;
    }
  }

  for (const auto & interface_name : start_interfaces) {
    const auto is_effort_interface = std::any_of(
      this->data_ptr_->joints.begin(),
      this->data_ptr_->joints.end(),
      [&interface_name](const JointData & joint) {
        return interface_name == joint.effort_interface_name;
      });
    if (is_effort_interface) {
      this->data_ptr_->effort_command_active = true;
    }
  }

  return hardware_interface::return_type::OK;
}

bool PandaGravityCompensatedSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, sim::Entity> & joints,
  const hardware_interface::HardwareInfo & hardware_info,
  sim::EntityComponentManager & ecm,
  int & /*update_rate*/)
{
  this->data_ptr_ = std::make_unique<PandaGravityCompensatedSystemPrivate>();
  this->nh_ = model_nh;
  this->data_ptr_->ecm = &ecm;
  this->data_ptr_->joints.resize(hardware_info.joints.size());
  this->data_ptr_->joint_to_chain_index.assign(hardware_info.joints.size(), -1);

  this->data_ptr_->robot_description_node =
    getHardwareParameter(hardware_info, "robot_description_node", "robot_state_publisher");
  this->data_ptr_->robot_description_param =
    getHardwareParameter(hardware_info, "robot_description_param", "robot_description");
  this->data_ptr_->root_link =
    getHardwareParameter(hardware_info, "root_link", "panda_link0");
  this->data_ptr_->tip_link =
    getHardwareParameter(hardware_info, "tip_link", "panda_hand");
  this->data_ptr_->gravity_vector = parseGravityVector(
    getHardwareParameter(hardware_info, "gravity_vector", "0 0 -9.81"));

  if (this->data_ptr_->joints.empty()) {
    RCLCPP_ERROR(this->nh_->get_logger(), "No joints were defined for the Panda Gazebo hardware");
    return false;
  }

  for (std::size_t index = 0; index < hardware_info.joints.size(); ++index) {
    const auto & joint_info = hardware_info.joints[index];
    auto & joint_data = this->data_ptr_->joints[index];
    joint_data.name = joint_info.name;
    joint_data.effort_interface_name =
      joint_info.name + "/" + hardware_interface::HW_IF_EFFORT;
    joint_data.position_interface_name =
      joint_info.name + "/" + hardware_interface::HW_IF_POSITION;

    const auto joint_it = joints.find(joint_info.name);
    if (joint_it == joints.end()) {
      RCLCPP_WARN(
        this->nh_->get_logger(),
        "Skipping joint '%s' because it was not found in the Gazebo model",
        joint_info.name.c_str());
      continue;
    }

    joint_data.sim_joint = joint_it->second;

    joint_data.joint_type =
      ecm.Component<sim::components::JointType>(joint_data.sim_joint)->Data();
    joint_data.joint_axis =
      ecm.Component<sim::components::JointAxis>(joint_data.sim_joint)->Data();

    if (!ecm.EntityHasComponentType(
          joint_data.sim_joint,
          sim::components::JointPosition().TypeId()))
    {
      ecm.CreateComponent(joint_data.sim_joint, sim::components::JointPosition());
    }

    if (!ecm.EntityHasComponentType(
          joint_data.sim_joint,
          sim::components::JointVelocity().TypeId()))
    {
      ecm.CreateComponent(joint_data.sim_joint, sim::components::JointVelocity());
    }

    if (!ecm.EntityHasComponentType(
          joint_data.sim_joint,
          sim::components::JointTransmittedWrench().TypeId()))
    {
      ecm.CreateComponent(joint_data.sim_joint, sim::components::JointTransmittedWrench());
    }

    double initial_position = std::numeric_limits<double>::quiet_NaN();
    double initial_velocity = std::numeric_limits<double>::quiet_NaN();
    double initial_effort = 0.0;

    for (const auto & state_interface : joint_info.state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_POSITION) {
        this->data_ptr_->state_interfaces.emplace_back(
          joint_info.name,
          hardware_interface::HW_IF_POSITION,
          &joint_data.joint_position);
        if (!state_interface.initial_value.empty()) {
          initial_position = std::stod(state_interface.initial_value);
          joint_data.joint_position = initial_position;
          joint_data.joint_position_cmd = initial_position;
        }
      } else if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
        this->data_ptr_->state_interfaces.emplace_back(
          joint_info.name,
          hardware_interface::HW_IF_VELOCITY,
          &joint_data.joint_velocity);
        if (!state_interface.initial_value.empty()) {
          initial_velocity = std::stod(state_interface.initial_value);
          joint_data.joint_velocity = initial_velocity;
        }
      } else if (state_interface.name == hardware_interface::HW_IF_EFFORT) {
        this->data_ptr_->state_interfaces.emplace_back(
          joint_info.name,
          hardware_interface::HW_IF_EFFORT,
          &joint_data.joint_effort);
        if (!state_interface.initial_value.empty()) {
          initial_effort = std::stod(state_interface.initial_value);
          joint_data.joint_effort = initial_effort;
        }
      }
    }

    for (const auto & command_interface : joint_info.command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_EFFORT) {
        joint_data.has_effort_command_interface = true;
        this->data_ptr_->command_interfaces.emplace_back(
          joint_info.name,
          hardware_interface::HW_IF_EFFORT,
          &joint_data.joint_effort_cmd);
        continue;
      }

      if (command_interface.name == hardware_interface::HW_IF_POSITION) {
        joint_data.has_position_command_interface = true;
        this->data_ptr_->command_interfaces.emplace_back(
          joint_info.name,
          hardware_interface::HW_IF_POSITION,
          &joint_data.joint_position_cmd);
        continue;
      }

      if (command_interface.name != hardware_interface::HW_IF_EFFORT) {
        RCLCPP_ERROR(
          this->nh_->get_logger(),
          "Joint '%s' requested unsupported command interface '%s'. "
          "This hardware plugin only supports effort and position commands.",
          joint_info.name.c_str(),
          command_interface.name.c_str());
        return false;
      }
    }

    if (!std::isnan(initial_position)) {
      ecm.CreateComponent(
        joint_data.sim_joint,
        sim::components::JointPositionReset({initial_position}));
    }

    if (!std::isnan(initial_velocity)) {
      ecm.CreateComponent(
        joint_data.sim_joint,
        sim::components::JointVelocityReset({initial_velocity}));
    }

    joint_data.joint_effort_cmd = initial_effort;
  }

  rclcpp::NodeOptions node_options;
  node_options.context(this->nh_->get_node_base_interface()->get_context());
  node_options.start_parameter_event_publisher(false);
  node_options.start_parameter_services(false);

  auto parameter_node = std::make_shared<rclcpp::Node>(
    "panda_gazebo_hardware_param_client",
    node_options);
  auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(
    parameter_node,
    this->data_ptr_->robot_description_node);

  if (!parameter_client->wait_for_service(10s)) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Timed out waiting for parameter services on '%s'",
      this->data_ptr_->robot_description_node.c_str());
    return false;
  }

  std::string robot_description;
  try {
    robot_description = parameter_client->get_parameter<std::string>(
      this->data_ptr_->robot_description_param);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Failed to read '%s' from '%s': %s",
      this->data_ptr_->robot_description_param.c_str(),
      this->data_ptr_->robot_description_node.c_str(),
      exception.what());
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromString(robot_description, tree)) {
    RCLCPP_ERROR(this->nh_->get_logger(), "Failed to parse robot_description into a KDL tree");
    return false;
  }

  KDL::Chain chain;
  if (!tree.getChain(this->data_ptr_->root_link, this->data_ptr_->tip_link, chain)) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Failed to build KDL chain from '%s' to '%s'",
      this->data_ptr_->root_link.c_str(),
      this->data_ptr_->tip_link.c_str());
    return false;
  }

  this->data_ptr_->chain = std::make_unique<KDL::Chain>(chain);
  this->data_ptr_->gravity_solver =
    std::make_unique<KDL::ChainDynParam>(*this->data_ptr_->chain, this->data_ptr_->gravity_vector);

  std::unordered_map<std::string, int> joint_name_to_index;
  joint_name_to_index.reserve(this->data_ptr_->joints.size());
  for (std::size_t index = 0; index < this->data_ptr_->joints.size(); ++index) {
    joint_name_to_index.emplace(this->data_ptr_->joints[index].name, static_cast<int>(index));
  }

  int chain_joint_index = 0;
  for (unsigned int segment_index = 0; segment_index < this->data_ptr_->chain->getNrOfSegments();
       ++segment_index)
  {
    const auto & joint = this->data_ptr_->chain->getSegment(segment_index).getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;
    }

    const auto it = joint_name_to_index.find(joint.getName());
    if (it == joint_name_to_index.end()) {
      RCLCPP_ERROR(
        this->nh_->get_logger(),
        "Joint '%s' from the KDL chain is not exported by ros2_control",
        joint.getName().c_str());
      return false;
    }

    this->data_ptr_->joint_to_chain_index[static_cast<std::size_t>(it->second)] = chain_joint_index;
    ++chain_joint_index;
  }

  if (chain_joint_index != static_cast<int>(this->data_ptr_->chain->getNrOfJoints())) {
    RCLCPP_ERROR(this->nh_->get_logger(), "Failed to map all KDL joints to Gazebo joints");
    return false;
  }

  RCLCPP_INFO(
    this->nh_->get_logger(),
    "Loaded Panda gravity compensation chain from '%s' to '%s' with %u joints",
    this->data_ptr_->root_link.c_str(),
    this->data_ptr_->tip_link.c_str(),
    this->data_ptr_->chain->getNrOfJoints());

  return true;
}

hardware_interface::return_type PandaGravityCompensatedSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  for (auto & joint_data : this->data_ptr_->joints) {
    if (joint_data.sim_joint == sim::kNullEntity) {
      continue;
    }

    const auto * joint_position =
      this->data_ptr_->ecm->Component<sim::components::JointPosition>(joint_data.sim_joint);
    const auto * joint_velocity =
      this->data_ptr_->ecm->Component<sim::components::JointVelocity>(joint_data.sim_joint);
    const auto * joint_wrench =
      this->data_ptr_->ecm->Component<sim::components::JointTransmittedWrench>(joint_data.sim_joint);

    if (!joint_position || !joint_velocity || !joint_wrench) {
      continue;
    }

    joint_data.joint_position = joint_position->Data()[0];
    joint_data.joint_velocity = joint_velocity->Data()[0];

    GZ_PHYSICS_NAMESPACE Vector3d force_or_torque;
    if (joint_data.joint_type == sdf::JointType::PRISMATIC) {
      force_or_torque = {
        joint_wrench->Data().force().x(),
        joint_wrench->Data().force().y(),
        joint_wrench->Data().force().z()};
    } else {
      force_or_torque = {
        joint_wrench->Data().torque().x(),
        joint_wrench->Data().torque().y(),
        joint_wrench->Data().torque().z()};
    }

    joint_data.joint_effort = force_or_torque.GZ_VECTOR_DOT(
      GZ_PHYSICS_NAMESPACE Vector3d{
        joint_data.joint_axis.Xyz()[0],
        joint_data.joint_axis.Xyz()[1],
        joint_data.joint_axis.Xyz()[2]});
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PandaGravityCompensatedSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  KDL::JntArray positions(this->data_ptr_->chain->getNrOfJoints());
  KDL::JntArray gravity(this->data_ptr_->chain->getNrOfJoints());

  for (std::size_t joint_index = 0; joint_index < this->data_ptr_->joints.size(); ++joint_index) {
    const int chain_index = this->data_ptr_->joint_to_chain_index[joint_index];
    if (chain_index < 0) {
      continue;
    }
    positions(static_cast<unsigned int>(chain_index)) =
      this->data_ptr_->joints[joint_index].joint_position;
  }

  if (this->data_ptr_->gravity_solver->JntToGravity(positions, gravity) < 0) {
    RCLCPP_ERROR(this->nh_->get_logger(), "Failed to compute Panda gravity compensation torques");
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t joint_index = 0; joint_index < this->data_ptr_->joints.size(); ++joint_index) {
    auto & joint_data = this->data_ptr_->joints[joint_index];
    if (joint_data.sim_joint == sim::kNullEntity) {
      continue;
    }

    if (joint_data.has_position_command_interface) {
      auto * joint_position_reset =
        this->data_ptr_->ecm->Component<sim::components::JointPositionReset>(joint_data.sim_joint);
      if (joint_position_reset == nullptr) {
        this->data_ptr_->ecm->CreateComponent(
          joint_data.sim_joint,
          sim::components::JointPositionReset({joint_data.joint_position_cmd}));
      } else {
        *joint_position_reset = sim::components::JointPositionReset({joint_data.joint_position_cmd});
      }
      continue;
    }

    const int chain_index = this->data_ptr_->joint_to_chain_index[joint_index];
    double total_effort = 0.0;
    if (this->data_ptr_->effort_command_active) {
      total_effort = joint_data.joint_effort_cmd;
    }
    if (chain_index >= 0) {
      total_effort += gravity(static_cast<unsigned int>(chain_index));
    }

    auto * joint_effort_cmd =
      this->data_ptr_->ecm->Component<sim::components::JointForceCmd>(joint_data.sim_joint);

    if (joint_effort_cmd == nullptr) {
      this->data_ptr_->ecm->CreateComponent(
        joint_data.sim_joint,
        sim::components::JointForceCmd({total_effort}));
    } else {
      *joint_effort_cmd = sim::components::JointForceCmd({total_effort});
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace panda_gazebo_hardware

PLUGINLIB_EXPORT_CLASS(
  panda_gazebo_hardware::PandaGravityCompensatedSystem,
  gz_ros2_control::GazeboSimSystemInterface)

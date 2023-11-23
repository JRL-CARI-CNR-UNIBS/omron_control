#include "omron_hardware_interface/omron_hardware_interface.hpp"

namespace omron {
OmronAria::OmronAria() :
//  m_client_connector(&m_args),
//  m_client_update(&m_client),
  get_pose_status__ftor(this, &OmronAria::get_pose_status__cb)
{
  #ifdef DEBUG_ON
  #include "rcutils/error_handling.h"
  if(rcutils_logging_set_logger_level(m_logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG))
  {
    RCLCPP_ERROR(m_logger, "Cannot set logger severity to DEBUG, using default");
  }
  #endif
}

hardware_interface::CallbackReturn
OmronAria::on_init(const hardware_interface::HardwareInfo& info)
{
  if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_FATAL(m_logger, "hardware_interface::SystemInterface::on_init: ERROR");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_DEBUG(m_logger, "system_interface parent: OK");

  m_twist__command.resize(info_.gpios.size(), std::numeric_limits<double>::quiet_NaN());
  m_twist__states.resize (info_.gpios.size(), std::numeric_limits<double>::quiet_NaN());
  m_pose__states.resize  (info_.gpios.size(), std::numeric_limits<double>::quiet_NaN());

  RCLCPP_DEBUG(m_logger, "Resizing: OK");

  for(const hardware_interface::ComponentInfo& gpio : info_.gpios)
  {
//    if(gpio.command_interfaces.size() > 1)
//    {
//      RCLCPP_FATAL(m_logger, "Expected only 1 command interface");
//      return hardware_interface::CallbackReturn::ERROR;
//    }
//    if(gpio.command_interfaces.size() == 0)
//    {
//      RCLCPP_WARN(m_logger, "No command interfaces defined for GPIO '%s'", gpio.name.c_str());
//    }
//    else if(!(gpio.command_interfaces[0].name == HW_FORWARD_VEL ||
//         gpio.command_interfaces[0].name == HW_TURN_VEL))
//    {
//      RCLCPP_FATAL(m_logger, "GPIO '%s' has %s command_interface. Expected %s or %s",
//                   gpio.name.c_str(), gpio.command_interfaces[0].name.c_str(),
//                   HW_FORWARD_VEL.c_str(), HW_TURN_VEL.c_str());
//      return hardware_interface::CallbackReturn::ERROR;
//    }

    auto velocity_interface__itor = std::find_if(info_.gpios.begin(), info_.gpios.end(), [this](const hardware_interface::ComponentInfo& in){return in.state_interfaces.size() == 2 &&
                                                                                                                                                    (in.state_interfaces.at(0).name == this->HW_FORWARD_VEL ||
                                                                                                                                                     in.state_interfaces.at(0).name == this->HW_TURN_VEL
                                                                                                                                                     ) && (
                                                                                                                                                     in.state_interfaces.at(1).name == this->HW_FORWARD_VEL ||
                                                                                                                                                     in.state_interfaces.at(1).name == this->HW_TURN_VEL);
                                                                                                                                             });
    if(velocity_interface__itor == info_.gpios.end())
    {
      RCLCPP_FATAL(m_logger, "Missing %s and %s command interfaces",
                   HW_FORWARD_VEL.c_str(), HW_TURN_VEL.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  // TODO: Test per controllare ci siano tutti i pezzi della posa
  RCLCPP_DEBUG(m_logger, "Various tests: OK");

  m_connection_data.ip =   info.hardware_parameters.at("ip_address");
  m_connection_data.port = info.hardware_parameters.at("port");
  m_connection_data.user = info.hardware_parameters.at("user");
  m_connection_data.pwd = "admin";
  m_prefix = info.hardware_parameters.at("prefix");
  m_hz = std::stoi(info.hardware_parameters.at("update_frequency"));
  m_max_vel.linear = std::stod(info.hardware_parameters.at("max_linear_vel"));
  m_max_vel.angular = std::stod(info.hardware_parameters.at("max_angular_vel"));

  RCLCPP_INFO(m_logger, "init ok");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OmronAria::export_state_interfaces()
{
  RCLCPP_DEBUG(m_logger, "Exporting states interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
// size_t forward_idx = std::distance(info_.gpios.begin(), std::find_if(info_.gpios.begin(), info_.gpios.end(), [this](const hardware_interface::ComponentInfo& in){return in.state_interfaces.at(0).name == this->HW_FORWARD_VEL;}));
// size_t turn_idx    = std::distance(info_.gpios.begin(), std::find_if(info_.gpios.begin(), info_.gpios.end(), [this](const hardware_interface::ComponentInfo& in){return in.state_interfaces.at(0).name == this->HW_TURN_VEL;}));
  auto velocity_interface__itor = std::find_if(info_.gpios.begin(),
                                               info_.gpios.end(),
                                               [this](const hardware_interface::ComponentInfo& in)
            {
              return in.name == m_prefix + "/" + hardware_interface::HW_IF_VELOCITY && in.state_interfaces.size() == 2;
            }
  );
  if (velocity_interface__itor == info_.gpios.end())
  {
    RCLCPP_FATAL(m_logger, "No. Exporting no state interface.");
    return state_interfaces;
  }
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      velocity_interface__itor->name, this->HW_FORWARD_VEL, &m_twist__states[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      velocity_interface__itor->name, this->HW_TURN_VEL, &m_twist__states[1]));

  auto pose_gpio__itor = std::find_if(info_.gpios.begin(), info_.gpios.end(), [this](const hardware_interface::ComponentInfo& in){return in.name == m_prefix + "/" + HW_IF_POSE;});
  if(pose_gpio__itor == info_.gpios.end())
  {
    RCLCPP_FATAL(m_logger, "Missing pose state_interfaces. Exporting error, returning no interfaces");
    return std::vector<hardware_interface::StateInterface>();
  }

  for(size_t idx = 0; idx < pose_gpio__itor->state_interfaces.size(); ++idx)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      pose_gpio__itor->name, pose_gpio__itor->state_interfaces.at(idx).name, &m_pose__states[idx]));
  }

  RCLCPP_DEBUG(m_logger, "Ending exporting states interfaces");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OmronAria::export_command_interfaces()
{
  RCLCPP_DEBUG(m_logger, "Exporting command interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
//  size_t forward_idx = std::distance(info_.gpios.begin(), std::find_if(info_.gpios.begin(), info_.gpios.end(), [this](const hardware_interface::ComponentInfo& in){return in.command_interfaces.at(0).name== this->HW_FORWARD_VEL;}));
//  size_t turn_idx    = std::distance(info_.gpios.begin(), std::find_if(info_.gpios.begin(), info_.gpios.end(), [this](const hardware_interface::ComponentInfo& in){return in.command_interfaces.at(0).name == this->HW_TURN_VEL;}));
  auto velocity_interface__itor = std::find_if(info_.gpios.begin(),
                                               info_.gpios.end(),
                                               [this](const hardware_interface::ComponentInfo& in)
            {
              return in.name == m_prefix + "/" + hardware_interface::HW_IF_VELOCITY && in.command_interfaces.size() == 2;
            }
  );
  if (velocity_interface__itor == info_.gpios.end())
  {
    RCLCPP_FATAL(m_logger, "No. Exporting no state interface.");
    return command_interfaces;
  }
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      velocity_interface__itor->name, this->HW_FORWARD_VEL, &m_twist__command[0]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      velocity_interface__itor->name, this->HW_TURN_VEL, &m_twist__command[1]));
  RCLCPP_DEBUG(m_logger, "Ending exporting command interfaces");
  return command_interfaces;

}

hardware_interface::CallbackReturn
OmronAria::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  Aria::init();
  m_client.enforceProtocolVersion("5MTX");
  m_args.addPlain("-host");
  m_args.addPlain(m_connection_data.ip.c_str());
  m_args.addPlain("-p");
  m_args.addPlain(m_connection_data.port.c_str());
  m_args.addPlain("-u");
  m_args.addPlain(m_connection_data.user.c_str());
  m_args.addPlain("-pwd");
  m_args.addPlain(m_connection_data.pwd.c_str());
  m_client_connector = std::make_shared<ArClientSimpleConnector>(&m_args);
  m_client_connector->parseArgs();
  RCLCPP_DEBUG(m_logger, "Connection parameter: OK");

//  m_client_update.requestUpdates(m_hz);
////  m_client_update.addUpdateCB(&get_pose_status__ftor);

  RCLCPP_INFO(m_logger, "Aria configuration: OK");
  //Connect
  if (!m_client_connector->connectClient(&m_client))
  {
    if (m_client.wasRejected())
      RCLCPP_FATAL_STREAM(m_logger, "Server" << m_client.getHost() << "rejected connection, exiting");
    else
      RCLCPP_FATAL_STREAM(m_logger, "Could not connect to server" << m_client.getHost() <<  "exiting");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  if(m_client.dataExists("updateNumbers"))
  {
    m_client.addHandler("updateNumbers", &get_pose_status__ftor);
    m_client.request("updateNumbers", 50);
  }
  else
  {
    RCLCPP_FATAL(m_logger, "'updateNumbers' request cannot be set. Exiting...");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  m_client.runAsync();
  RCLCPP_INFO(m_logger, "Connected to server");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OmronAria::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OmronAria::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  set_cmd_vel(0.0, 0.0);
  RCLCPP_WARN(m_logger, "Send STOP");
  if(m_client.isConnected())
  {
    m_client.stopRunning();
    m_client.disconnect();
    RCLCPP_WARN(m_logger, "Disconnecting from robot");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
OmronAria::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  if(m_client.getRunning())
  {
//    m_client_update.lock();
//    m_pose__states[0]  = m_client_update.getX();
//    m_pose__states[1]  = m_client_update.getY();
//    m_pose__states[2]  = m_client_update.getTh();
//    m_twist__states[0] = m_client_update.getVel();
//    m_twist__states[1] = m_client_update.getRotVel();
//    m_client_update.unlock();
    m_pose__states[0] = m_status_data.pose.x;
    m_pose__states[1] = m_status_data.pose.y;
    m_pose__states[2] = m_status_data.pose.rz;
    m_twist__states[0] = m_status_data.vel.x;
    m_twist__states[1] = m_status_data.vel.rz;
  }
  else
  {
    RCLCPP_ERROR(m_logger, "Aria client is not running");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OmronAria::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  set_cmd_vel(m_twist__command[0], m_twist__command[1]);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OmronAria::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                            const std::vector<std::string>& stop_interfaces)
{
  RCLCPP_ERROR(m_logger, "Function perform_command_mode_switch() not implemented.");
  return hardware_interface::return_type::OK;
}

/**
 * Private members and utility functions
 */

void OmronAria::get_pose_status__cb(ArNetPacket *packet)
{
  m_status_data.battery_voltage =   ( (double) packet->bufToByte2() )/10.0;
  m_status_data.pose.x =            (  (double) packet->bufToByte4() )/1000.0;
  m_status_data.pose.y =            (  (double) packet->bufToByte4() )/1000.0;
  m_status_data.pose.rz =             (double) packet->bufToByte2();
  m_status_data.vel.x =             (  (double) packet->bufToByte2() )/1000.0;
  m_status_data.vel.rz =              (double) packet->bufToByte2();
  m_status_data.vel.y =             (  (double) packet->bufToByte2() )/1000.0;
  m_status_data.temperature =         (double) packet->bufToByte();
}

void OmronAria::set_cmd_vel(const double& forward, const double& turn)
{
  double fw = forward / m_max_vel.linear;
  double tr = (turn*180.0/M_PI) / m_max_vel.angular;
  if(fabs(forward) > 0.001 || fabs(turn) > 0.001)
  {
    ArNetPacket packet;
    m_is_cmd_valid = true;
    packet.doubleToBuf(100 * fw);
    packet.doubleToBuf(100 * tr);
    packet.doubleToBuf(100);
    packet.doubleToBuf(0.0);
    m_client.requestOnce("ratioDrive", &packet);
  }
  else
  {
    m_is_cmd_valid = false;
  }
}


} // omron

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(omron::OmronAria, hardware_interface::SystemInterface)

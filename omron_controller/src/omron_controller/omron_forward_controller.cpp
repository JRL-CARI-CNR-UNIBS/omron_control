#include "omron_controller/omron_forward_controller.hpp"

#include <controller_interface/helpers.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace omron {
  controller_interface::CallbackReturn OmronController::on_init()
  {
    #ifdef DEBUG_ON
    #include "rcutils/error_handling.h"
    if(rcutils_logging_set_logger_level(this->get_node()->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG))
    {
      RCLCPP_ERROR(this->get_node()->get_logger(), "Cannot set logger severity to DEBUG, using default");
    }
    #endif
    try
    {
      m_param_listener = std::make_shared<omron_controller::ParamListener>(get_node());
      m_params = m_param_listener->get_params();
    }
    catch (const std::exception & e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration
  OmronController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    command_interfaces_config.names = m_command_interface_names;

    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration
  OmronController::state_interface_configuration() const
  {
//    return controller_interface::InterfaceConfiguration{
//        controller_interface::interface_configuration_type::ALL};
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = m_state_interface_names;
    return state_interfaces_config;
  }

  controller_interface::CallbackReturn
  OmronController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    m_params = m_param_listener->get_params();
    cmd_vel_topic = m_params.cmd_vel_topic;

    // Publisher, subscribers, tf
    m_cmd_vel__sub = this->get_node()->create_subscription<geometry_msgs::msg::Twist>(
                       cmd_vel_topic,
                       rclcpp::SystemDefaultsQoS(),
                       [this](const geometry_msgs::msg::Twist::SharedPtr msg)
                       {
                          m_rt_buffer__ptr.writeFromNonRT(msg);
                       });

    m_odom__pub = this->get_node()->create_publisher<nav_msgs::msg::Odometry>(
                    odom_topic,
                    rclcpp::SystemDefaultsQoS()
                    );

    m_tf__broad = std::make_unique<tf2_ros::TransformBroadcaster>(this->get_node());

    // state interfaces
    std::vector<std::string> state_interface_names;
    state_interface_names = m_params.interfaces.velocity.states;
    std::transform(state_interface_names.begin(),
                   state_interface_names.end(),
                   state_interface_names.begin(),
                   [this](const std::string& s){
      return m_params.interfaces.velocity.prefix + "/" + s;
    });
    m_state_interface_names = state_interface_names;
    state_interface_names = m_params.interfaces.pose.states;
    std::transform(state_interface_names.begin(),
                   state_interface_names.end(),
                   state_interface_names.begin(),
                   [this](const std::string& s){
      return m_params.interfaces.pose.prefix + "/" + s;
    });
    m_state_interface_names.insert(m_state_interface_names.end(),
                                   state_interface_names.begin(),
                                   state_interface_names.end());
    state_interfaces_.reserve(m_state_interface_names.size());

    // command interfaces
    m_command_interface_names = m_params.interfaces.velocity.command;
    for(std::string& name : m_command_interface_names)
      name = m_params.interfaces.velocity.prefix + "/" + name;
    command_interfaces_.reserve(m_command_interface_names.size());
    RCLCPP_INFO(this->get_node()->get_logger(), "configure successful");

    // Reference interfaces
    m_reference_interface_names = m_params.interfaces.reference.command;
    for(std::string& name : m_reference_interface_names)
      name = m_params.interfaces.reference.prefix + "/" + name;
    reference_interfaces_.resize(m_reference_interface_names.size(), std::numeric_limits<double>::quiet_NaN());
    RCLCPP_DEBUG(this->get_node()->get_logger(), "reference configure successful");

    m_use_open_loop = m_params.feedback.use_open_loop;
    m_kp = m_params.feedback.kp;

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  OmronController::on_activate(const rclcpp_lifecycle::State& /**/)
  {
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
    if(
       !controller_interface::get_ordered_interfaces(
         command_interfaces_, m_command_interface_names, std::string(""), ordered_interfaces)
         || m_command_interface_names.size() != ordered_interfaces.size())
    {
      RCLCPP_ERROR(
            this->get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
            m_command_interface_names.size(), ordered_interfaces.size());
      return controller_interface::CallbackReturn::ERROR;
    }
//    if(m_reference_interface_names.size() != 2)
//    {
//      RCLCPP_ERROR(this->get_node()->get_logger(), "Expected 2 reference interfaces.");
//      return controller_interface::CallbackReturn::ERROR;
//    }

    m_rt_buffer__ptr = realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist::SharedPtr>(nullptr);
    RCLCPP_INFO(this->get_node()->get_logger(), "activate successful");

    std::fill(
        reference_interfaces_.begin(), reference_interfaces_.end(),
        std::numeric_limits<double>::quiet_NaN());
    RCLCPP_INFO(this->get_node()->get_logger(), "reference activate successful");

    for(auto& s : state_interfaces_)
      RCLCPP_WARN_STREAM(this->get_node()->get_logger(), "State interfaces: " << s.get_name());

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  OmronController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // reset command buffer
    m_rt_buffer__ptr = realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist::SharedPtr>(nullptr);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  bool OmronController::on_set_chained_mode(bool chained_mode)
  {
    if(chained_mode)
    {
      m_cmd_vel__sub = nullptr;
    }
    else
    {
      m_cmd_vel__sub = this->get_node()->create_subscription<geometry_msgs::msg::Twist>(
                         cmd_vel_topic,
                         rclcpp::SystemDefaultsQoS(),
                         [this](const geometry_msgs::msg::Twist::SharedPtr msg)
                         {
                            m_rt_buffer__ptr.writeFromNonRT(msg);
                         });
    }
    return true;
  }

  controller_interface::return_type
  OmronController::update_and_write_commands(const rclcpp::Time&, const rclcpp::Duration& duration)
  {
    // Command
    if(std::isnan(reference_interfaces_.at(0)) || std::isnan(reference_interfaces_.at(1)))
    {
      RCLCPP_ERROR(this->get_node()->get_logger(), "NaN reference, stopping");     
      command_interfaces_.at(0).set_value(0.0);
      command_interfaces_.at(1).set_value(0.0);
      return controller_interface::return_type::ERROR;
    }

    double reference[2];
    reference[0] = reference_interfaces_.at(0);
    reference[1] = reference_interfaces_.at(1);
//    reference[0] = std::fabs(reference[0]) > 0.01? reference[0] : 0.0; // m/s
//    reference[1] = std::fabs(reference[1]) > 0.01? reference[1] : 0.0; // rad/s
    command_interfaces_.at(0).set_value(reference[0]);
    command_interfaces_.at(1).set_value(reference[1]);

    return controller_interface::return_type::OK;
  }

  std::vector<hardware_interface::CommandInterface>
  OmronController::on_export_reference_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> reference_interfaces;
    for(size_t idx = 0; idx < m_reference_interface_names.size(); ++idx)
    {
      reference_interfaces.push_back(hardware_interface::CommandInterface(
        this->get_node()->get_name(), m_reference_interface_names.at(idx), &reference_interfaces_[idx]));
    }

    return reference_interfaces;
  }

  controller_interface::return_type
  OmronController::update_reference_from_subscribers()
  {
    geometry_msgs::msg::Twist::SharedPtr* cmd_vel = m_rt_buffer__ptr.readFromRT();
    if (!(!cmd_vel || !(*cmd_vel)))
    {
      if(!std::isnan((*cmd_vel)->linear.x) && !std::isnan((*cmd_vel)->angular.z))
      {
        reference_interfaces_.at(0) = (*cmd_vel)->linear.x;
        reference_interfaces_.at(1) = (*cmd_vel)->angular.z;
      }
      else
      {
        RCLCPP_ERROR_THROTTLE(this->get_node()->get_logger(), *(this->get_node()->get_clock()), 1000, "Twist message is has NaN");
        return controller_interface::return_type::ERROR;
      }
    }
    else
    {
      RCLCPP_ERROR_THROTTLE(this->get_node()->get_logger(), *(this->get_node()->get_clock()), 1000, "Cannot receive Twist message");
      return controller_interface::return_type::ERROR;
    }
    return controller_interface::return_type::OK;
  }
}

PLUGINLIB_EXPORT_CLASS(omron::OmronController, controller_interface::ChainableControllerInterface)

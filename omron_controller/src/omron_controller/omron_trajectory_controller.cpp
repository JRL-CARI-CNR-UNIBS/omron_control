#include "omron_controller/omron_trajectory_controller.hpp"

#include "controller_interface/helpers.hpp"

namespace omron {
  controller_interface::CallbackReturn OmronPositionController::on_init()
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
      m_param_listener = std::make_shared<omron_position_controller::ParamListener>(get_node());
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
  OmronPositionController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    command_interfaces_config.names = m_command_interface_names;

    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration
  OmronPositionController::state_interface_configuration() const
  {
//    return controller_interface::InterfaceConfiguration{
//        controller_interface::interface_configuration_type::ALL};
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = m_state_interface_names;
    return state_interfaces_config;
  }

  controller_interface::CallbackReturn
  OmronPositionController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    m_params = m_param_listener->get_params();
    m_topics.ff_vel = m_params.cmd_vel_topic;
    m_topics.ref_pos = m_params.cmd_pos_topic;
    m_k1 = m_params.feedback.k1;
    m_k2 = m_params.feedback.k2;
    m_k3 = m_params.feedback.k3;

    // Publisher, subscribers, tf
    m_ff_vel__sub = this->get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(m_topics.ff_vel, 10, [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
       m_rt_buffer_vel__ptr.writeFromNonRT(msg);
    });
    m_ref_pos__sub = this->get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(m_topics.ref_pos, 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
       m_rt_buffer_pos__ptr.writeFromNonRT(msg);
    });


    m_tf__buffer = std::make_unique<tf2_ros::Buffer>(this->get_node()->get_clock());
    m_tf__listener = std::make_unique<tf2_ros::TransformListener>(*m_tf__buffer);

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
    if(m_params.interfaces.reference.command.size() != 5)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Reference names should be 5: x,y,theta,u_lin,u_ang. Given: %ld", m_params.interfaces.reference.command.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    m_reference_interface_names = m_params.interfaces.reference.command;
    for(std::string& name : m_reference_interface_names)
      name = m_params.interfaces.reference.prefix + "/" + name;
    reference_interfaces_.resize(m_reference_interface_names.size(), std::numeric_limits<double>::quiet_NaN());
    RCLCPP_DEBUG(this->get_node()->get_logger(), "reference configure successful");

//    m_use_open_loop = m_params.feedback.use_open_loop;
//    m_kp = m_params.feedback.kp;

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  OmronPositionController::on_activate(const rclcpp_lifecycle::State& /**/)
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

    m_rt_buffer_vel__ptr = realtime_tools::RealtimeBuffer<geometry_msgs::msg::TwistStamped::SharedPtr>(nullptr);
    m_rt_buffer_pos__ptr = realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseStamped::SharedPtr>(nullptr);

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
  OmronPositionController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // reset command buffer
    m_rt_buffer_vel__ptr = realtime_tools::RealtimeBuffer<geometry_msgs::msg::TwistStamped::SharedPtr>(nullptr);
    m_rt_buffer_pos__ptr = realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseStamped::SharedPtr>(nullptr);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  bool OmronPositionController::on_set_chained_mode(bool chained_mode)
  {
    if(chained_mode)
    {
      m_ff_vel__sub = nullptr;
      m_ref_pos__sub = nullptr;
    }
    else
    {
      m_ff_vel__sub = this->get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
                         m_topics.ff_vel,
                         rclcpp::SystemDefaultsQoS(),
                         [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
                         {
                            m_rt_buffer_vel__ptr.writeFromNonRT(msg);
                         });
      m_ref_pos__sub = this->get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
                         m_topics.ref_pos,
                         rclcpp::SystemDefaultsQoS(),
                         [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
                         {
                            m_rt_buffer_pos__ptr.writeFromNonRT(msg);
                         });
    }
    return true;
  }

  std::vector<hardware_interface::CommandInterface>
  OmronPositionController::on_export_reference_interfaces()
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
  OmronPositionController::update_reference_from_subscribers()
  {
    // FIXME: TO CHANGE!
    geometry_msgs::msg::TwistStamped::SharedPtr* ff_vel = m_rt_buffer_vel__ptr.readFromRT();
    geometry_msgs::msg::PoseStamped::SharedPtr* ref_pos = m_rt_buffer_pos__ptr.readFromRT();

    if (!(!ref_pos || !(*ref_pos)))
    {
      if(!std::isnan((*ref_pos)->pose.position.x)
         && !std::isnan((*ref_pos)->pose.position.y)
         && !std::isnan((*ref_pos)->pose.orientation.z))
      {
        reference_interfaces_.at(0) = (*ref_pos)->pose.position.x;
        reference_interfaces_.at(1) = (*ref_pos)->pose.position.y;
        reference_interfaces_.at(2) = (*ref_pos)->pose.orientation.z;
      }
      else
      {
        RCLCPP_ERROR_THROTTLE(this->get_node()->get_logger(), *(this->get_node()->get_clock()), 1000, "Message has NaN.");
        return controller_interface::return_type::ERROR;
      }
    }
    else
    {
      RCLCPP_ERROR_THROTTLE(this->get_node()->get_logger(), *(this->get_node()->get_clock()), 1000, "Cannot receive PoseStamped message");
      return controller_interface::return_type::ERROR;
    }

    if (!(!ff_vel || !(*ff_vel)))
    {
      if(!std::isnan((*ff_vel)->twist.linear.x) && !std::isnan((*ff_vel)->twist.angular.z))
      {
        reference_interfaces_.at(3) = (*ff_vel)->twist.linear.x;
        reference_interfaces_.at(4) = (*ff_vel)->twist.angular.z;
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

  controller_interface::return_type
  OmronPositionController::update_and_write_commands(const rclcpp::Time&, const rclcpp::Duration &)
  {
    constexpr double margin {1e-3};
    const double z1 = state_interfaces_.at(0).get_value() - reference_interfaces_.at(0);
    const double z2 = state_interfaces_.at(1).get_value() - reference_interfaces_.at(1);
    double th_e = (state_interfaces_.at(2).get_value() - reference_interfaces_.at(2));
    double z3 = std::tan(th_e);

    if(std::cos(th_e)< margin)
    {
      th_e = (th_e)/std::abs(th_e) * M_PI/2.0 * 0.95;
      z3 = std::tan(th_e);
    }

    command_interfaces_.at(0).set_value((reference_interfaces_.at(4) - m_k1 * std::fabs(reference_interfaces_.at(3))*(z1+z2*z3)) / std::cos(th_e));
    command_interfaces_.at(1).set_value((-m_k2*state_interfaces_.at(4).get_value()*z2 - m_k3*std::fabs(state_interfaces_.at(3).get_value())*z3) * std::pow(std::cos(th_e),2) + reference_interfaces_.at(4));
    if(command_interfaces_.at(0).get_value() == 0)
    {
      // TODO: usa altra tecnica di controllo.
    }
    return controller_interface::return_type::OK;

  }

} // omron

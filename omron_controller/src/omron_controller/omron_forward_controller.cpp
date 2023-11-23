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

    m_status_vel__pub = this->get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
                          status_twist_topic,
                          rclcpp::SystemDefaultsQoS());
    m_status_pose__pub = this->get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
                           status_pose_topic,
                           rclcpp::SystemDefaultsQoS());
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

    if(m_params.tf.use_tf)
    {
      geometry_msgs::msg::TransformStamped tf_odom;
      tf_odom.header.frame_id = m_params.tf.from;
      tf_odom.child_frame_id = "odom";
      tf_odom.header.stamp = this->get_node()->get_clock()->now();
      tf_odom.transform.translation.x = 0.0;
      tf_odom.transform.translation.y = 0.0;
      tf_odom.transform.translation.z = 0.0;
      tf_odom.transform.rotation.x = 0.0;
      tf_odom.transform.rotation.y = 0.0;
      tf_odom.transform.rotation.z = 0.0;
      tf_odom.transform.rotation.w = 1.0;
      auto static_tf__broad = tf2_ros::StaticTransformBroadcaster(this->get_node());
      static_tf__broad.sendTransform(tf_odom);
      RCLCPP_WARN(this->get_node()->get_logger(), "Static odom published");
    }

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
  OmronController::update_and_write_commands(const rclcpp::Time&, const rclcpp::Duration &)
  {
    // Status
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->get_node()->get_clock()->now();
    twist_msg.twist.linear.x = state_interfaces_.at(0).get_value();
    twist_msg.twist.angular.z = state_interfaces_.at(1).get_value();
    m_status_vel__pub->publish(twist_msg);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = twist_msg.header.stamp;
    pose_msg.pose.position.x = state_interfaces_.at(2).get_value();
    pose_msg.pose.position.y = state_interfaces_.at(3).get_value();
    tf2::Quaternion quat({0.0,0.0,1.0},state_interfaces_.at(4).get_value() * M_PI/180.0);
//    pose_msg.pose.orientation.z = std::sin(state_interfaces_.at(4).get_value()/2.0);
//    pose_msg.pose.orientation.w = std::cos(state_interfaces_.at(4).get_value()/2.0);
    pose_msg.pose.orientation = tf2::toMsg(quat);
    m_status_pose__pub->publish(pose_msg);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = twist_msg.header;
    odom_msg.pose.pose = pose_msg.pose;
    odom_msg.twist.twist = twist_msg.twist;
    m_odom__pub->publish(odom_msg);

    if(m_params.tf.use_tf)
    {
      geometry_msgs::msg::TransformStamped tf_odom;
      tf_odom.header.frame_id = m_params.tf.from;
      tf_odom.child_frame_id = m_params.tf.odom;
      tf_odom.header.stamp = this->get_node()->get_clock()->now();
      tf_odom.transform.translation.x = 0.0;
      tf_odom.transform.translation.y = 0.0;
      tf_odom.transform.translation.z = 0.0;
      tf_odom.transform.rotation.x = 0.0;
      tf_odom.transform.rotation.y = 0.0;
      tf_odom.transform.rotation.z = 0.0;
      tf_odom.transform.rotation.w = 1.0;
      m_tf__broad->sendTransform(tf_odom);

      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = twist_msg.header.stamp;
      tf_msg.header.frame_id = m_params.tf.odom; //m_params.tf.from;
      tf_msg.child_frame_id = m_params.tf.to;
      tf_msg.transform.translation.x = pose_msg.pose.position.x;
      tf_msg.transform.translation.y = pose_msg.pose.position.y;
      tf_msg.transform.rotation.z = pose_msg.pose.orientation.z;
      tf_msg.transform.rotation.w = pose_msg.pose.orientation.w;
      m_tf__broad->sendTransform(tf_msg);
    }

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
    reference[0] = std::fabs(reference[0]) > 0.01? reference[0] : 0.0; // m/s
    reference[1] = std::fabs(reference[1]) > 0.01? reference[1] : 0.0; // deg/s
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

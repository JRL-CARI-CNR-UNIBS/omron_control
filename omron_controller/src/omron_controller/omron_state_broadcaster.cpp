#include "omron_controller/omron_state_broadcaster.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace omron {
controller_interface::CallbackReturn OmronStateBroadcaster::on_init()
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
    m_param_listener = std::make_shared<omron_state_broadcaster::ParamListener>(get_node());
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
OmronStateBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
OmronStateBroadcaster::state_interface_configuration() const
{
//    return controller_interface::InterfaceConfiguration{
//        controller_interface::interface_configuration_type::ALL};
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = m_state_interface_names;
  return state_interfaces_config;
}

controller_interface::CallbackReturn
OmronStateBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  m_odom__pub = this->get_node()->create_publisher<nav_msgs::msg::Odometry>(K_ODOM_TOPIC, 10);

  m_tf__broad = std::make_unique<tf2_ros::TransformBroadcaster>(*(this->get_node()));

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

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
OmronStateBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_WARN(this->get_node()->get_logger(), "on_activate() not implemented. Skipped");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
OmronStateBroadcaster::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Status
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = this->get_node()->get_clock()->now();
  twist_msg.twist.linear.x = state_interfaces_.at(0).get_value();
  twist_msg.twist.angular.z = state_interfaces_.at(1).get_value();

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = twist_msg.header.stamp;
  pose_msg.pose.position.x = state_interfaces_.at(2).get_value();
  pose_msg.pose.position.y = state_interfaces_.at(3).get_value();
  tf2::Quaternion quat({0.0,0.0,1.0},state_interfaces_.at(4).get_value() * M_PI/180.0);
  //    pose_msg.pose.orientation.z = std::sin(state_interfaces_.at(4).get_value()/2.0);
  //    pose_msg.pose.orientation.w = std::cos(state_interfaces_.at(4).get_value()/2.0);
  pose_msg.pose.orientation = tf2::toMsg(quat);

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
  return controller_interface::return_type::OK;
}

} // namespace omron

PLUGINLIB_EXPORT_CLASS(omron::OmronStateBroadcaster, controller_interface::ControllerInterface);

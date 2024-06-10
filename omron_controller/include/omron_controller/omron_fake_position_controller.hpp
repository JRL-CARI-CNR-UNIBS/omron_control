#ifndef OMRON_FAKE_POSITION_CONTROLLER_HPP
#define OMRON_FAKE_POSITION_CONTROLLER_HPP

#include <controller_interface/chainable_controller_interface.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "omron_fake_position_controller_parameters.hpp"

//#include della libreria parametri

namespace omron {
class OmronFakePositionController : public controller_interface::ChainableControllerInterface
{

public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;


  bool on_set_chained_mode(bool chained_mode) override;


  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type update_reference_from_subscribers() override;

  std::shared_ptr<omron_fake_position_controller::ParamListener> m_param_listener;
  omron_fake_position_controller::Params m_params;

  bool m_use_open_loop;
  double m_kp;

  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist::SharedPtr> m_rt_buffer__ptr;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseStamped::SharedPtr> m_rt_buffer_pose__ptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel__sub;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom__pub;

  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf__broad;

  std::vector<std::string> m_state_interface_names,
                           m_command_velocity_interface_names,
                          m_command_position_interface_names,
                           m_reference_interface_names;

  std::string status_twist_topic {"status/velocity"};
  std::string status_pose_topic {"status/pose"};
  std::string odom_topic {"/odom"};
  std::string cmd_vel_topic;
};
}

#endif // OMRON_FAKE_POSITION_CONTROLLER_HPP

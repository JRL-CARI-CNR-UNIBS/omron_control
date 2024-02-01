#ifndef OMRON_STATE_BROADCASTER_HPP
#define OMRON_STATE_BROADCASTER_HPP

#include <rclcpp/rclcpp.hpp>

#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "omron_state_broadcaster_parameters.hpp"

namespace omron {
class OmronStateBroadcaster : public controller_interface::ControllerInterface
{

public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

//  controller_interface::CallbackReturn on_deactivate(
//    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<omron_state_broadcaster::ParamListener> m_param_listener;
  omron_state_broadcaster::Params m_params;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom__pub;

  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf__broad;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf_bcast;

  std::vector<std::string> m_state_interface_names,
                           m_command_interface_names;

  static constexpr char K_ODOM_TOPIC[] {"/omron/odom"};
};
}

#endif // OMRON_STATE_BROADCASTER_HPP

#ifndef OMRON_POSITION_CONTROLLER_HPP
#define OMRON_POSITION_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
//#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "omron_trajectory_controller_parameters.hpp"

/*
 * Implements "Tracking of a Reference Vehicle with the Same Kinematics" controller
 * from Springer Handbook of Robotics, pp. 1242-1243
 */

namespace omron {
class OmronPositionController : public controller_interface::ChainableControllerInterface
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

  std::shared_ptr<omron_position_controller::ParamListener> m_param_listener;
  omron_position_controller::Params m_params;

  realtime_tools::RealtimeBuffer<geometry_msgs::msg::TwistStamped::SharedPtr> m_rt_buffer_vel__ptr;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseStamped::SharedPtr> m_rt_buffer_pos__ptr;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr m_ff_vel__sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_ref_pos__sub;

  std::unique_ptr<tf2_ros::TransformListener> m_tf__listener;
  std::unique_ptr<tf2_ros::Buffer> m_tf__buffer;

  std::vector<std::string> m_state_interface_names,
                           m_command_interface_names,
                           m_reference_interface_names;


  struct TopicNames{
    std::string ff_vel;
    std::string ref_pos;
  } m_topics;

  double m_k1, m_k2, m_k3;
};
}

#endif // OMRON_POSITION_CONTROLLER_HPP

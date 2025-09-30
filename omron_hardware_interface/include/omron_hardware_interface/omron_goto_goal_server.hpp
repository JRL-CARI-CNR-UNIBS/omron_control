#ifndef OMRON_GOTO_GOAL_SERVER_HPP
#define OMRON_GOTO_GOAL_SERVER_HPP

#include <omron_msgs/action/goto_goal.hpp>

#include <rclcpp/rclcpp.hpp>
#include <Aria/Aria.h>
#include <ArNetworking/ArNetworking.h>
#include <ArNetworking/ArClientHandlerRobotUpdate.h>
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <angles/angles.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>



class OmronGotoGoalServer : public rclcpp::Node
{
public:
  OmronGotoGoalServer(ArClientBase* t_client);

//  std::vector<std::string> get_goal_names();
  std::string get_actual_goal();
private:
  using GotoGoalAction = omron_msgs::action::GotoGoal;
  using GoalHandleGotoGoalAction = rclcpp_action::ServerGoalHandle<GotoGoalAction>;

  ArClientBase* m_ar_client;
  ArClientHandlerRobotUpdate m_ar_client_update;
  
  std::vector<std::string> m_goals;

  // For gotoPose
  std::string m_base_frame;
  std::string m_map_frame;

  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_list;
  bool m_enable_goto_pose;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map__sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom__sub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_goal_pose__pub;
  std::optional<nav_msgs::msg::OccupancyGrid> m_map_data;
  std::optional<nav_msgs::msg::Odometry> m_odom_data;

  rclcpp_action::Server<GotoGoalAction>::SharedPtr goto_action_server_;


  ArFunctor1C<OmronGotoGoalServer, ArNetPacket*> m_get_actual_goal__ftor;
  ArFunctor1C<OmronGotoGoalServer, ArNetPacket*> m_get_goal_list__ftor;
  void get_actual_goal__cb();
  void get_goal_list__cb();
  void handle_map_data__cb(const nav_msgs::msg::OccupancyGrid& msg);
  void handle_odom_data__cb(const nav_msgs::msg::Odometry& msg);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GotoGoalAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGotoGoalAction> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleGotoGoalAction> goal_handle);
  
    void execute_goto(const std::shared_ptr<GoalHandleGotoGoalAction> goal_handle);

  void sleep_for_async(std::chrono::nanoseconds sleep_duration);

};

#endif // OMRON_GOTO_GOAL_SERVER_HPP

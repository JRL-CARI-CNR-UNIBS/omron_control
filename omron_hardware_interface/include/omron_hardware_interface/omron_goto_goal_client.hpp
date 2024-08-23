#ifndef OMRON_GOTO_GOAL_CLIENT_HPP
#define OMRON_GOTO_GOAL_CLIENT_HPP

#include <omron_msgs/srv/goto_goal.hpp>
#include <omron_msgs/srv/goto_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Aria/Aria.h>
#include <ArNetworking/ArNetworking.h>
#include <ArNetworking/ArClientHandlerRobotUpdate.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>



class OmronGotoGoalClient : public rclcpp::Node
{
public:
  OmronGotoGoalClient(ArClientBase* t_client);

  void go_to_goal__cb(const omron_msgs::srv::GotoGoal::Request::SharedPtr request,
                            omron_msgs::srv::GotoGoal::Response::SharedPtr response);

  void go_to_pose__cb(const omron_msgs::srv::GotoPose::Request::SharedPtr request,
                            omron_msgs::srv::GotoPose::Response::SharedPtr response);
//  std::vector<std::string> get_goal_names();
  std::string get_actual_goal();
private:
  ArClientBase* m_client;

  std::vector<std::string> m_goals;

  // For gotoPose
  std::string m_base_frame;
  std::string m_map_frame;

  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_list;
  bool m_enable_goto_pose;

  rclcpp::Service<omron_msgs::srv::GotoGoal>::SharedPtr m_goal__srv;
  rclcpp::Service<omron_msgs::srv::GotoPose>::SharedPtr m_pose__srv;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map__sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom__sub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_goal_pose__pub;
  std::optional<nav_msgs::msg::OccupancyGrid> m_map_data;
  std::optional<nav_msgs::msg::Odometry> m_odom_data;



  ArFunctor1C<OmronGotoGoalClient, ArNetPacket*> m_get_actual_goal__ftor;
  ArFunctor1C<OmronGotoGoalClient, ArNetPacket*> m_get_goal_list__ftor;
  void get_actual_goal__cb();
  void get_goal_list__cb();
  void handle_map_data__cb(const nav_msgs::msg::OccupancyGrid& msg);
  void handle_odom_data__cb(const nav_msgs::msg::Odometry& msg);

};

#endif // OMRON_GOTO_GOAL_CLIENT_HPP

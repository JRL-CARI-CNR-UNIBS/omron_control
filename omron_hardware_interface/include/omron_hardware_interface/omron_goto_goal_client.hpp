#ifndef OMRON_GOTO_GOAL_CLIENT_HPP
#define OMRON_GOTO_GOAL_CLIENT_HPP

#include <omron_msgs/srv/goto_goal.hpp>
#include <omron_msgs/srv/goto_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Aria/Aria.h>
#include <ArNetworking/ArNetworking.h>
#include <ArNetworking/ArClientHandlerRobotUpdate.h>


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

  rclcpp::Service<omron_msgs::srv::GotoGoal>::SharedPtr m_goal__srv;

  ArFunctor1C<OmronGotoGoalClient, ArNetPacket*> m_get_actual_goal__ftor;
  ArFunctor1C<OmronGotoGoalClient, ArNetPacket*> m_get_goal_list__ftor;
  void get_actual_goal__cb();
  void get_goal_list__cb();

};

#endif // OMRON_GOTO_GOAL_CLIENT_HPP

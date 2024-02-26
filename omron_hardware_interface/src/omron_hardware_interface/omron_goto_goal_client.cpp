#include "omron_hardware_interface/omron_goto_goal_client.hpp"

using namespace std::chrono_literals;

OmronGotoGoalClient::OmronGotoGoalClient(ArClientBase* t_client)
  : rclcpp::Node("omron_goal_client"),
    m_client(t_client)
{
  m_goal__srv = this->create_service<omron_msgs::srv::GotoGoal>("goto_goal",
                                                                std::bind(&OmronGotoGoalClient::go_to_goal__cb,
                                                                          this,
                                                                          std::placeholders::_1,
                                                                          std::placeholders::_2));
//  m_client->addHandler("goalName", &m_get_actual_goal__ftor);
//  m_client->addHandler("getGoals", &m_get_goal_list__ftor);

}

void OmronGotoGoalClient::go_to_goal__cb(const omron_msgs::srv::GotoGoal::Request::SharedPtr request,
                                               omron_msgs::srv::GotoGoal::Response::SharedPtr response)
{
  // TODO: da rendere un'action
//  if(m_goals.empty())
//  {
//    m_client->requestOnce("getGoals");
//  }
  ArClientHandlerRobotUpdate ar_client_update(m_client);
  m_client->requestOnceWithString("gotoGoal", request->goal.c_str());
  ar_client_update.requestUpdates(20);
  this->get_clock()->sleep_for(1s);
  std::string mode, status;
  while(m_client->getRunningWithLock())
  {
    ar_client_update.lock();
    mode = ar_client_update.getMode();
    status = ar_client_update.getStatus();
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Mode: " << ar_client_update.getMode() << "\n" <<
                                            "Status: " << ar_client_update.getStatus());
    if(mode == "Goal seeking")
    {
      if(status == "Arrived at " + request->goal)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "Arrived at " << request->goal);
        break;
      }
      else if(status == "Going to " + request->goal)
      {
        // In corso
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Something went wrong");
        response->result = omron_msgs::srv::GotoGoal::Response::FAILED;
        return;
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Goal request rejected");
      response->result = omron_msgs::srv::GotoGoal::Response::FAILED;
      return;
    }
    ar_client_update.unlock();
    this->get_clock()->sleep_for(1s);
  }
  response->result = omron_msgs::srv::GotoGoal::Response::ARRIVED;
}

void OmronGotoGoalClient::go_to_pose__cb(const omron_msgs::srv::GotoPose::Request::SharedPtr request, omron_msgs::srv::GotoPose::Response::SharedPtr response)
{
    ArClientHandlerRobotUpdate ar_client_update(m_client);
    ArNetPacket packet;
    ArTypes::
    packet.
    m_client->requestOnceWithString("gotoPose", request->goal.c_str());
    ar_client_update.requestUpdates(20);
    this->get_clock()->sleep_for(1s);
    std::string mode, status;
    while(m_client->getRunningWithLock())
    {
      ar_client_update.lock();
      mode = ar_client_update.getMode();
      status = ar_client_update.getStatus();
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Mode: " << ar_client_update.getMode() << "\n" <<
                                              "Status: " << ar_client_update.getStatus());
      if(mode == "Goal seeking")
      {
        if(status == "Arrived at " + request->goal)
        {
          RCLCPP_INFO_STREAM(this->get_logger(), "Arrived at " << request->goal);
          break;
        }
        else if(status == "Going to " + request->goal)
        {
          // In corso
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Something went wrong");
          response->result = omron_msgs::srv::GotoGoal::Response::FAILED;
          return;
        }
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Goal request rejected");
        response->result = omron_msgs::srv::GotoGoal::Response::FAILED;
        return;
      }
      ar_client_update.unlock();
      this->get_clock()->sleep_for(1s);
    }
    response->result = omron_msgs::srv::GotoGoal::Response::ARRIVED;
}

#include "omron_hardware_interface/omron_goto_goal_client.hpp"

#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

OmronGotoGoalClient::OmronGotoGoalClient(ArClientBase* t_client)
  : rclcpp::Node("omron_goal_client"),
    m_client(t_client)
{
  this->declare_parameter("base_frame", rclcpp::PARAMETER_STRING);
  this->declare_parameter("map_frame", rclcpp::PARAMETER_STRING);
  this->declare_parameter("map", rclcpp::PARAMETER_STRING);
  this->declare_parameter("odom", rclcpp::PARAMETER_STRING);

  m_enable_goto_pose = true;

  if(!this->get_parameter("base_frame", m_base_frame))
  {
    RCLCPP_ERROR(this->get_logger(), "Missing 'base_frame' parameter, %s/goto_pose service disabled", this->get_name());
    m_enable_goto_pose = false;
  }
  if(!this->get_parameter("map_frame", m_map_frame))
  {
    RCLCPP_ERROR(this->get_logger(), "Missing 'map_frame' parameter, %s/goto_pose service disabled", this->get_name());
    m_enable_goto_pose = false;
  }
  std::string map_topic;
  if(!this->get_parameter("map", map_topic))
  {
    RCLCPP_ERROR(this->get_logger(), "Missing 'map' parameter, %s/goto_pose service disabled", this->get_name());
    m_enable_goto_pose = false;
  }
  // std::string odometry_topic;
  // if(!this->get_parameter("odom", odometry_topic))
  // {
  //   RCLCPP_ERROR(this->get_logger(), "Missing 'odom' parameter, %s/goto_pose service disabled", this->get_name());
  //   m_enable_goto_pose = false;
  // }


  m_goal__srv = this->create_service<omron_msgs::srv::GotoGoal>("goto_goal",
                                                                std::bind(&OmronGotoGoalClient::go_to_goal__cb,
                                                                          this,
                                                                          std::placeholders::_1,
                                                                          std::placeholders::_2));
  if(m_enable_goto_pose)
  {
    m_map__sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, rclcpp::QoS(1).transient_local(), std::bind(&OmronGotoGoalClient::handle_map_data__cb,
                                                                                          this,
                                                                                          std::placeholders::_1));

    // m_odom__sub = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic, 1, std::bind(&OmronGotoGoalClient::handle_odom_data__cb,
                                                                                            //  this,
                                                                                            //  std::placeholders::_1));

    m_pose__srv = this->create_service<omron_msgs::srv::GotoPose>("goto_pose", std::bind(&OmronGotoGoalClient::go_to_pose__cb, this, std::placeholders::_1, std::placeholders::_2));
  }

  m_goal_pose__pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("omron_goal", 1);

  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_list = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);

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
    RCLCPP_WARN_STREAM(this->get_logger(), "Mode: " << ar_client_update.getMode() << "\n" <<
                                            "Status: " << ar_client_update.getStatus());
    if(status[0] == 'A')
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Arrived at Goal");
      break;
    }
    else if(status[0] == 'G')
    {
      // In corso
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Something went wrong");
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

  if(!m_map_data.has_value())
  {
    RCLCPP_ERROR(this->get_logger(), "No map found. Aborting.");
    return;
  }
//if(!m_odom_data.has_value())  {
//   RCLCPP_ERROR(this->get_logger(), "No odometry msg received. Aborting.");
//   return;
// }

//  if(m_tf_buffer->canTransform(request->goal.header.frame_id, m_base_frame, tf2::TimePointZero, 1s))
//  {
//    RCLCPP_ERROR(this->get_logger(), "No transform from %s to %s found. Aborting.", request->goal.header.frame_id.c_str(), m_base_frame.c_str());
//    return;
//  }

  Eigen::Affine3d T_map_goal;
  // WARNING: do per scontato che il frame ros e l'origine della mappa coincidano
  if(request->from_frame == omron_msgs::srv::GotoPose::Request::FROM_WORLD)
  {
    Eigen::Affine3d T_world_goal;
    tf2::fromMsg(request->goal.pose,T_world_goal);
    geometry_msgs::msg::TransformStamped tf = m_tf_buffer->lookupTransform(request->goal.header.frame_id, m_map_frame, tf2::TimePointZero, 1s);
    Eigen::Affine3d T_world_map;
    T_world_map = tf2::transformToEigen(tf);
    T_map_goal = T_world_map.inverse() * T_world_goal;
  }
  else if(request->from_frame == omron_msgs::srv::GotoPose::Request::FROM_MAP)
  {
    tf2::fromMsg(request->goal.pose, T_map_goal);
  }
  else
  {
    response->result = omron_msgs::srv::GotoPose::Response::FAILED;
    return;
  }
  // Publish pose to debug
  geometry_msgs::msg::PoseStamped goal_msg;
  goal_msg.header.stamp = this->get_clock()->now();
  goal_msg.header.frame_id = m_map_frame;
  goal_msg.pose = tf2::toMsg(T_map_goal);
  m_goal_pose__pub->publish(goal_msg);

  ArNetPacket packet;

  const ArTypes::Byte4 ar_x =  std::round(T_map_goal.translation()(0)/m_map_data.value().info.resolution * 1e3);
  const ArTypes::Byte4 ar_y =  std::round(T_map_goal.translation()(1)/m_map_data.value().info.resolution * 1e3);
  const ArTypes::Byte2 ar_th = std::round(Eigen::AngleAxisd(T_map_goal.linear()).angle()/m_map_data.value().info.resolution * 1e3);
  packet.byte4ToBuf(ar_x);
  packet.byte4ToBuf(ar_y);
  packet.byte2ToBuf(ar_th);

  m_client->requestOnce("gotoPose", &packet);
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

    if(status[0] == 'A')
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Arrived at Goal");
      break;
    }
    else if(status[0] == 'G')
    {
      // In corso
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Something went wrong");
      response->result = omron_msgs::srv::GotoGoal::Response::FAILED;
      return;
    }

    ar_client_update.unlock();
    this->get_clock()->sleep_for(50ms);
  }
  response->result = omron_msgs::srv::GotoGoal::Response::ARRIVED;
}

void OmronGotoGoalClient::handle_map_data__cb(const nav_msgs::msg::OccupancyGrid& msg)
{
  m_map_data = std::make_optional(msg);
}

void OmronGotoGoalClient::handle_odom_data__cb(const nav_msgs::msg::Odometry &msg)
{
  m_odom_data = std::make_optional(msg);
}

#include "omron_hardware_interface/omron_goto_goal_server.hpp"

#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

OmronGotoGoalServer::OmronGotoGoalServer(ArClientBase* t_client)
  : rclcpp::Node("omron_goal_client"),
    m_ar_client(t_client),
    m_ar_client_update(m_ar_client)
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
  std::string odometry_topic;
  if(!this->get_parameter("odom", odometry_topic))
  {
    RCLCPP_ERROR(this->get_logger(), "Missing 'odom' parameter, %s/goto_pose service disabled", this->get_name());
    m_enable_goto_pose = false;
  }

  
  goto_action_server_ = rclcpp_action::create_server<GotoGoalAction>(
    this,
    "goto_goal",
    std::bind(&OmronGotoGoalServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&OmronGotoGoalServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&OmronGotoGoalServer::handle_accepted, this, std::placeholders::_1));

  if(m_enable_goto_pose)
  {
    m_map__sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, rclcpp::QoS(1).transient_local(), std::bind(&OmronGotoGoalServer::handle_map_data__cb,
                                                                                          this,
                                                                                          std::placeholders::_1));

    m_odom__sub = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic, 1, std::bind(&OmronGotoGoalServer::handle_odom_data__cb,
                                                                                             this,
                                                                                             std::placeholders::_1));
  }

  m_goal_pose__pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("omron_goal", 1);

  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_list = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);

}

rclcpp_action::GoalResponse
OmronGotoGoalServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const GotoGoalAction::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with goal frame id: %s", goal->goal.header.frame_id.c_str());
  (void)uuid;

  // ArClientHandlerRobotUpdate ar_client_update(m_ar_client);

  if(!m_map_data.has_value())
  {
    RCLCPP_ERROR(this->get_logger(), "No map found. Aborting.");
    return rclcpp_action::GoalResponse::REJECT;
  }


  // You could reject a goal request here
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
OmronGotoGoalServer::handle_cancel(
  const std::shared_ptr<GoalHandleGotoGoalAction> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  
  if(!m_map_data.has_value())
  {
    RCLCPP_ERROR(this->get_logger(), "No map found. I cannot execute the goal.");
    return rclcpp_action::CancelResponse::REJECT;
  }
  
  (void)goal_handle;
  // You could reject a cancel request here
  return rclcpp_action::CancelResponse::ACCEPT;
}

void OmronGotoGoalServer::sleep_for_async(std::chrono::nanoseconds sleep_duration)
{
  auto start = this->now();
  rclcpp::Rate rate(10);
  while((this->now() - start) < rclcpp::Duration(sleep_duration)) {
    rate.sleep();
  }
}

void
OmronGotoGoalServer::handle_accepted(
  const std::shared_ptr<GoalHandleGotoGoalAction> goal_handle)
{

  std::thread{std::bind(&OmronGotoGoalServer::execute_goto, this, std::placeholders::_1), goal_handle}.detach();
}

void 
OmronGotoGoalServer::execute_goto(const std::shared_ptr<GoalHandleGotoGoalAction> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<GotoGoalAction::Feedback>();
  auto result = std::make_shared<GotoGoalAction::Result>();
  
  ar_client_update.requestUpdates(20);
  sleep_for_async(std::chrono::nanoseconds(100ms));

  Eigen::Affine3d T_world_goal, T_world_map, T_map_goal;

  tf2::fromMsg(goal->goal.pose,T_map_goal);

  auto start_time = this->now();
  while(not(m_tf_buffer->canTransform(goal->goal.header.frame_id, m_map_frame, tf2::TimePointZero, 1s)) 
        and (this->now() - start_time) < rclcpp::Duration(3s)) {
    RCLCPP_WARN(this->get_logger(), "Waiting for transform from %s to %s", goal->goal.header.frame_id.c_str(), m_map_frame.c_str());
    if(goal_handle->is_canceling()) {
      result->result = GotoGoalAction::Result::GOING_TO;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      m_ar_client_update.stopUpdates();
      return;
    }
  }
  geometry_msgs::msg::TransformStamped tf_world_map;
  try{
    tf_world_map = m_tf_buffer->lookupTransform(goal->goal.header.frame_id, m_map_frame, tf2::TimePointZero, 1s);
  }
  catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform not available: %s", ex.what());
    return;
  }
  T_world_map = tf2::transformToEigen(tf_world_map);
  RCLCPP_INFO(this->get_logger(), "Transform from %s to %s found.", goal->goal.header.frame_id.c_str(), m_map_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "Transform from %s to %s : x: %f, y: %f, z: %f", goal->goal.header.frame_id.c_str(), m_map_frame.c_str(),
              T_world_map.translation()(0),
              T_world_map.translation()(1),
              T_world_map.translation()(2));

  T_map_goal = T_world_map.inverse() * T_world_goal;

  ArNetPacket packet;

  const ArTypes::Byte4 ar_x =  std::round(T_map_goal.translation()(0)/m_map_data.value().info.resolution * 1e3);
  const ArTypes::Byte4 ar_y =  std::round(T_map_goal.translation()(1)/m_map_data.value().info.resolution * 1e3);
  const ArTypes::Byte2 ar_th = std::round(Eigen::AngleAxisd(T_map_goal.linear()).angle()/m_map_data.value().info.resolution * 1e3);
  packet.byte4ToBuf(ar_x);
  packet.byte4ToBuf(ar_y);
  packet.byte2ToBuf(ar_th);

  m_client->requestOnce("gotoPose", &packet);
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
      result->result = GotoGoalAction::Result::ARRIVED;
      goal_handle->succeed(result);
      break;
    }
    else if(status[0] == 'G')
    {
      // Going to goal
      feedback->current_status = GotoGoalAction::Feedback::GOING_TO;
      goal_handle->publish_feedback(feedback);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Something went wrong");
      result->result = GotoGoalAction::Result::FAILED;
      goal_handle->abort(result);
      break;
    }
    ar_client_update.unlock();
  }

  m_ar_client_update.stopUpdates();
  // result->result = GotoGoalAction::Result::ARRIVED;
  // goal_handle->succeed(result);
  // RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}


// void OmronGotoGoalServer::go_to_pose__cb(const omron_msgs::srv::GotoPose::Request::SharedPtr request, omron_msgs::srv::GotoPose::Response::SharedPtr response)
// {
//   ArClientHandlerRobotUpdate ar_client_update(m_ar_client);

//   if(!m_map_data.has_value())
//   {
//     RCLCPP_ERROR(this->get_logger(), "No map found. Aborting.");
//     return;
//   }


//   Eigen::Affine3d T_map_goal;
//   // WARNING: do per scontato che il frame ros e l'origine della mappa coincidano
//   if(request->from_frame == omron_msgs::srv::GotoPose::Request::FROM_WORLD)
//   {
//     Eigen::Affine3d T_world_goal;
//     tf2::fromMsg(request->goal.pose,T_world_goal);
//     geometry_msgs::msg::TransformStamped tf = m_tf_buffer->lookupTransform(request->goal.header.frame_id, m_map_frame, tf2::TimePointZero, 1s);
//     Eigen::Affine3d T_world_map;
//     T_world_map = tf2::transformToEigen(tf);
//     T_map_goal = T_world_map.inverse() * T_world_goal;
//   }
//   else if(request->from_frame == omron_msgs::srv::GotoPose::Request::FROM_MAP)
//   {
//     tf2::fromMsg(request->goal.pose, T_map_goal);
//   }
//   else
//   {
//     response->result = omron_msgs::srv::GotoPose::Response::FAILED;
//     return;
//   }
//   // Publish pose to debug
//   geometry_msgs::msg::PoseStamped goal_msg;
//   goal_msg.header.stamp = this->get_clock()->now();
//   goal_msg.header.frame_id = m_map_frame;
//   goal_msg.pose = tf2::toMsg(T_map_goal);
//   m_goal_pose__pub->publish(goal_msg);

//   ArNetPacket packet;

//   const ArTypes::Byte4 ar_x =  std::round(T_map_goal.translation()(0)/m_map_data.value().info.resolution * 1e3);
//   const ArTypes::Byte4 ar_y =  std::round(T_map_goal.translation()(1)/m_map_data.value().info.resolution * 1e3);
//   const ArTypes::Byte2 ar_th = std::round(Eigen::AngleAxisd(T_map_goal.linear()).angle()/m_map_data.value().info.resolution * 1e3);
//   packet.byte4ToBuf(ar_x);
//   packet.byte4ToBuf(ar_y);
//   packet.byte2ToBuf(ar_th);

//   m_ar_client->requestOnce("gotoPose", &packet);
//   ar_client_update.requestUpdates(20);
//   this->get_clock()->sleep_for(1s);
//   std::string mode, status;
//   while(m_ar_client->getRunningWithLock())
//   {
//     ar_client_update.lock();
//     mode = ar_client_update.getMode();
//     status = ar_client_update.getStatus();
//     RCLCPP_DEBUG_STREAM(this->get_logger(), "Mode: " << ar_client_update.getMode() << "\n" <<
//                                             "Status: " << ar_client_update.getStatus());

//     if(status[0] == 'A')
//     {
//       RCLCPP_INFO_STREAM(this->get_logger(), "Arrived at Goal");
//       break;
//     }
//     else if(status[0] == 'G')
//     {
//       // In corso
//     }
//     else
//     {
//       RCLCPP_ERROR(this->get_logger(), "Something went wrong");
//       response->result = omron_msgs::srv::GotoGoal::Response::FAILED;
//       return;
//     }

//     ar_client_update.unlock();
//     this->get_clock()->sleep_for(50ms);
//   }
//   response->result = omron_msgs::srv::GotoGoal::Response::ARRIVED;
// }

void OmronGotoGoalServer::handle_map_data__cb(const nav_msgs::msg::OccupancyGrid& msg)
{
  m_map_data = std::make_optional(msg);
}

void OmronGotoGoalServer::handle_odom_data__cb(const nav_msgs::msg::Odometry &msg)
{
  m_odom_data = std::make_optional(msg);
}

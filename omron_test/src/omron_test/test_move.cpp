#include "omron_msgs/srv/goto_goal.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/executors.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int16.hpp>

#include <eigen3/Eigen/Core>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>


#include <iostream>
#include <chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;

#include "rcutils/error_handling.h"

#define M_PI 3.14159265358979323846

class TestNode : public rclcpp::Node
{
public:
  TestNode()
    : rclcpp::Node("omron_test_node")
  {
    if(rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG))
    {
      RCLCPP_ERROR(get_logger(), "Cannot set logger severity to DEBUG, using default");
    }

    RCLCPP_WARN(get_logger(), "Creating TestNode");
    this->declare_parameter("cmd_vel_topic",       rclcpp::ParameterType::PARAMETER_STRING);
//    this->declare_parameter("pose_topic",          rclcpp::ParameterType::PARAMETER_STRING);
    this->declare_parameter("movements",           rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);
    this->declare_parameter("values",              rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("max_vel_in_m",        rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("max_acc_in_m",        rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("max_rot_vel_in_deg",  rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("max_rot_acc_in_deg",  rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("update_period_in_ms", rclcpp::ParameterType::PARAMETER_INTEGER);
    this->declare_parameter("motion_law", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("motion_law.max_vel_in_m",        rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("motion_law.max_acc_in_m",        rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("motion_law.max_rot_vel_in_deg",  rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("motion_law.max_rot_acc_in_deg",  rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);

    this->declare_parameter("use_close_loop",      rclcpp::ParameterType::PARAMETER_BOOL);

    // Publisher and subscription
    m_topics["cmd_vel"] = this->get_parameter("cmd_vel_topic").as_string();
    m_cmd_vel__pub = this->create_publisher<geometry_msgs::msg::Twist>(m_topics["cmd_vel"], 1);
    bag__pub = this->create_publisher<std_msgs::msg::Int16>("omron_test_number", 1);

    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);


    // Navigation action client
    m_client__action = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/omron/navigate_to_pose");


    // Parameters setup
    std::vector<long> movements = this->get_parameter("movements").as_integer_array();
    m_values = this->get_parameter("values").as_double_array();
    m_max_vel =     this->get_parameter("max_vel_in_m").as_double();
    m_max_acc =     this->get_parameter("max_acc_in_m").as_double();
    double max_rot_vel_in_deg = this->get_parameter("max_rot_vel_in_deg").as_double(); /* deg/s */
    double max_rot_acc_in_deg = this->get_parameter("max_rot_acc_in_deg").as_double(); /* deg/s^2 */
    m_max_rot_vel = max_rot_vel_in_deg*M_PI/180.0;
    m_max_rot_acc = max_rot_acc_in_deg*M_PI/180.0;
    m_dt_in_ns = std::chrono::nanoseconds(this->get_parameter("update_period_in_ms").as_int() * 1000000ul);
    m_dt = m_dt_in_ns.count()/1.0e9;
    m_motion_vel =     this->get_parameter("motion_law.max_vel_in_m").as_double_array();
    m_motion_acc =     this->get_parameter("motion_law.max_acc_in_m").as_double_array();
    m_motion_rot_vel = this->get_parameter("motion_law.max_rot_vel_in_deg").as_double_array(); /* deg/s */
    m_motion_rot_acc = this->get_parameter("motion_law.max_rot_acc_in_deg").as_double_array(); /* deg/s^2 */
    if (!((m_motion_vel.size() == m_motion_acc.size()) && (m_motion_rot_vel.size() == m_motion_rot_acc.size()) && (m_motion_vel.size() == m_motion_rot_acc.size()))) {
        RCLCPP_ERROR(get_logger(), "Motion law has different sizes. Ending...");
        return;
    }


    m_use_close_loop = this->get_parameter("use_close_loop").as_bool();

    if(movements.size() == 0)
    {
      RCLCPP_ERROR(get_logger(), "No movements specified. Ending...");
      return;
    }

    m_movements.resize(movements.size());
    std::transform(movements.begin(), movements.end(), m_movements.begin(),[](const long d){
      return d == 0? MoveType::LINEAR : MoveType::ANGULAR;
    });
    if (m_movements.size() != m_values.size())
    {
      RCLCPP_FATAL(this->get_logger(), "Parameters movements and value size are different! Exiting...");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Start in ");
    rclcpp::sleep_for(1s);
    RCLCPP_INFO(this->get_logger(), "...3");
    rclcpp::sleep_for(1s);
    RCLCPP_INFO(this->get_logger(), "...2");
    rclcpp::sleep_for(1s);
    RCLCPP_INFO(this->get_logger(), "...1");
    rclcpp::sleep_for(1s);
    RCLCPP_INFO(this->get_logger(), "Starting!");

    if(!this->set_home())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupting...");
      throw std::runtime_error("Missing home");
    }

    this->on_main();
  }

  void on_main()
  {
    if(!m_is_at_home)
    {
      if(!this->go_home__aria())
      {
        RCLCPP_ERROR(this->get_logger(), "Closing...");
        return;
      }
    }
    for(unsigned int n_idx = 0; n_idx < m_motion_vel.size(); ++n_idx){
        RCLCPP_DEBUG_STREAM(get_logger(), "Entering motion number " << n_idx);

        // Publish bag
        std_msgs::msg::Int16 bag_msg;
        bag_msg.data = n_idx + 1;
        bag__pub->publish(bag_msg);

        const double max_vel=m_motion_vel.at(n_idx);
        const double max_acc=m_motion_acc.at(n_idx);
        const double max_rot_vel=m_motion_rot_vel.at(n_idx)*M_PI/180.0; // conversion from deg/s to rad/s
        const double max_rot_acc=m_motion_rot_acc.at(n_idx)*M_PI/180.0; // conversion from deg/s^2 to rad/s^2

        const double t_la = max_vel/max_acc;
        const double t_ra = max_rot_vel/max_rot_acc;
        for(size_t idx = 0; idx < m_movements.size(); ++idx)
        {
          m_move_type = m_movements.at(idx);
          double val;
          if (m_move_type == MoveType::ANGULAR){
              val=m_values.at(idx)*M_PI/180.0; // conversion from deg/s to rad/s
          }
          else {
              val=m_values.at(idx);
          }

          RCLCPP_DEBUG_STREAM(get_logger(),
                              "Movement: " << (m_move_type == MoveType::LINEAR? "Linear" : "Angular") << "\n"<<
                              "Value: " << val);
          double t_a, qp_max, qpp_max;
          if(m_move_type == MoveType::LINEAR)
          {
            t_a =     t_la;
            qp_max =  max_vel;
            qpp_max = max_acc;
          }
          else
          {
            t_a =     t_ra;
            qp_max =  max_rot_vel;
            qpp_max = max_rot_acc;
          }
          double duration = val/qp_max+t_a;
          double t = 0.0;

          double qp = 0.0; // q = 0.0;
          geometry_msgs::msg::Twist cmd_msg;
          cmd_msg.linear.x = 0.0;
          cmd_msg.linear.y = 0.0;
          cmd_msg.angular.x = 0.0;
          cmd_msg.angular.y = 0.0;

          rclcpp::Rate rate(m_dt_in_ns);

          RCLCPP_DEBUG(get_logger(),"Starting movement");
          std::chrono::time_point t_start = std::chrono::steady_clock::now();
          while(t < duration + duration/20)
          {
            rate.reset();
            if(t < t_a)
            {
              qp = qpp_max * t;
//              q  = 0.5*qpp_max*std::pow(t,2);
            }
            else if(t_a <= t && t <= duration - t_a)
            {
              qp = qp_max;// qpp_max*t_a;
//              q = qpp_max*t_a*(t - 0.5*t_a);
            }
            else if(t < duration)
            {
              qp = qpp_max * (duration - t);
//              q = val - 0.5*qpp_max*std::pow((duration - t), 2);
            }
            else
            {
              qp = 0;
//              q = val;
            }

            t += m_dt;

    //        qp /= qp_max;
            if(m_move_type == MoveType::LINEAR)
            {
              cmd_msg.linear.x = qp;
              cmd_msg.angular.z = 0.0;
            }
            else
            {
              cmd_msg.linear.x = 0.0;
              cmd_msg.angular.z = qp;
            }
//            cmd_msg.header.stamp = this->get_clock()->now();
            m_cmd_vel__pub->publish(cmd_msg);
    //        m_cmd_pos__pub->publish()
            rate.sleep();
          }
          std::chrono::time_point t_end = std::chrono::steady_clock::now();
          RCLCPP_INFO_STREAM(this->get_logger(),
                             "MOTION END\n" <<
                             "Control Loop duration: " << (t_end - t_start).count()/1.0e9 << "\n" <<
                             "Acceleration ramp: " << t_a << "\n" <<
                             "Motion law duration: " << duration << "\n" <<
                             "Planned space to travel: " << val
                             );
        } // ending single movement
        RCLCPP_INFO_STREAM(get_logger(), "Movement number " << n_idx << " completed.");
        m_is_at_home = false;
        if(!m_is_at_home)
        {
          this->go_home__aria();
        }
    } // ending motions cycle
    RCLCPP_INFO(get_logger(), "All motions are completed.");
  }

protected:
  enum class MoveType{
    LINEAR = 0,
    ANGULAR = 1
  };
  MoveType m_move_type;

  std::vector<MoveType> m_movements;
  std::vector<double> m_values;
  double m_max_vel;
  double m_max_acc;
  double m_max_rot_vel;
  double m_max_rot_acc;
  std::vector<double> m_motion_vel;
  std::vector<double> m_motion_acc;
  std::vector<double> m_motion_rot_vel;
  std::vector<double> m_motion_rot_acc;
  std::chrono::nanoseconds m_dt_in_ns;
  double m_dt;

  bool m_use_close_loop;

  std::unordered_map<std::string, std::string> m_topics;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel__pub;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr bag__pub;
//  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose__sub;
  tf2_ros::Buffer::SharedPtr m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  geometry_msgs::msg::PoseStamped m_pose__msg;
  geometry_msgs::msg::PoseStamped m_home;
  bool m_is_home_set {false};
  bool m_is_at_home {false};

/*
 * Nav2
 */

public:
  using Nav2Pose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav2Pose = rclcpp_action::ClientGoalHandle<Nav2Pose>;
  bool set_home()
  {
//    rclcpp::sleep_for(500ms); // per aggiornare il messaggio
    geometry_msgs::msg::Twist start_msg;
    start_msg.linear.x = 0.0;
    start_msg.linear.y = 0.0;
    start_msg.linear.z = 0.0;
    start_msg.angular.x = 0.0;
    start_msg.angular.y = 0.0;
    start_msg.angular.z = 0.0;
    m_cmd_vel__pub->publish(start_msg);
    this->get_clock()->sleep_for(1s);

    auto now = this->get_clock()->now();
    if(m_tf_buffer->canTransform("map", "base_link", now, 5s))
    {
//      m_home = m_pose__msg;
      geometry_msgs::msg::TransformStamped tf = m_tf_buffer->lookupTransform("map", "base_link", now, 5s);
      m_home.header.frame_id = "map";
      m_home.header.stamp = tf.header.stamp;
      m_home.pose.position.x = tf.transform.translation.x;
      m_home.pose.position.y = tf.transform.translation.y;
      m_home.pose.position.z = tf.transform.translation.z;
      m_home.pose.orientation.x = tf.transform.rotation.x;
      m_home.pose.orientation.y = tf.transform.rotation.y;
      m_home.pose.orientation.z = tf.transform.rotation.z;
      m_home.pose.orientation.w = tf.transform.rotation.w;

      auto static_tf__bcast = tf2_ros::TransformBroadcaster(this);
      tf.child_frame_id = "home";
      static_tf__bcast.sendTransform(tf);

      m_is_home_set = true;
      m_is_at_home = true;
      RCLCPP_INFO(this->get_logger(), "Home set");
      RCLCPP_DEBUG_STREAM(this->get_logger(), "from: " << tf.header.frame_id << "\n" <<
                                              "x: " << m_home.pose.position.x << "\n" <<
                                              "y: " << m_home.pose.position.y << "\n" <<
                                              "z: " << m_home.pose.position.z << "\n" <<
                                              "w: " << m_home.pose.orientation.w << "\n" <<
                                              "x: " << m_home.pose.orientation.x << "\n" <<
                                              "y: " << m_home.pose.orientation.y << "\n" <<
                                              "z: " << m_home.pose.orientation.z);
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Cannot set home: missing transform");
      return false;
    }
  }

  bool go_home()
  {
    if (!this->m_client__action->wait_for_action_server(5s))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Go home action called");

    auto send_goal_options = rclcpp_action::Client<Nav2Pose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&TestNode::goal_result__cb, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&TestNode::feedback__cb, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&TestNode::result__cb, this, _1);

    Nav2Pose::Goal goal_msg;
    goal_msg.pose = m_home;
    goal_msg.pose.header.stamp = this->get_clock()->now();

    auto future_goal_handle = this->m_client__action->async_send_goal(goal_msg, send_goal_options);

    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Action failed");
    }
    auto res__future = m_client__action->async_get_result(future_goal_handle.get());
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), res__future);
    RCLCPP_INFO(this->get_logger(), "Go home action terminated");
    return true;
  }

  bool go_home__aria()
  {
    RCLCPP_INFO(get_logger(), "Requesting goto goal - 1");
    m_goal__clnt = this->create_client<omron_msgs::srv::GotoGoal>("/goto_goal");
    std::string aria_goal = "Home";
    omron_msgs::srv::GotoGoal::Request::SharedPtr goal = std::make_shared<omron_msgs::srv::GotoGoal::Request>();
    goal->goal = aria_goal;

    RCLCPP_INFO(get_logger(), "Requesting goto goal - 2");
    auto goal_res__future = m_goal__clnt->async_send_request(goal);
//    goal_res__future.wait();
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_res__future);
    if(goal_res__future.get()->result == omron_msgs::srv::GotoGoal::Response::FAILED)
    {
      RCLCPP_FATAL(this->get_logger(), "Cannot return home. Stopping...");
//      throw std::runtime_error("Aria navigation failed");
      return false;
    }
    return true;
  }

  bool is_at_home(){return m_is_at_home;}

protected:
  rclcpp::Client<omron_msgs::srv::GotoGoal>::SharedPtr m_goal__clnt;
  rclcpp_action::Client<Nav2Pose>::SharedPtr m_client__action;
  void goal_result__cb(const GoalHandleNav2Pose::SharedPtr& goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }
  void feedback__cb(
    GoalHandleNav2Pose::SharedPtr,
    const std::shared_ptr<const Nav2Pose::Feedback> feedback)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "distance_remaining: %f", feedback->distance_remaining);
  }
  void result__cb(const GoalHandleNav2Pose::WrappedResult & /*result*/)
  {
    RCLCPP_INFO(this->get_logger(), "Return home completed");
    m_is_at_home = true;
  }

};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>();
//  rclcpp::spin_some(node);
  rclcpp::sleep_for(1s);
  RCLCPP_WARN(rclcpp::get_logger("motion_test"), "Shutting down");
  rclcpp::shutdown();
  return 0;
}

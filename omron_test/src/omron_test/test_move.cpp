#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/executors.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <iostream>
#include <chrono>

using namespace std::chrono_literals;

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
    this->declare_parameter("movements",           rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);
    this->declare_parameter("values",              rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("max_vel_in_m",        rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("max_acc_in_m",        rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("max_rot_vel_in_deg",  rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("max_rot_acc_in_deg",  rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("update_period_in_ms", rclcpp::ParameterType::PARAMETER_INTEGER);

    this->declare_parameter("use_close_loop",      rclcpp::ParameterType::PARAMETER_BOOL);

    // Publisher and subscription
    m_topics["cmd_vel"] =    this->get_parameter("cmd_vel_topic").as_string();
    m_cmd_vel__pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(m_topics["cmd_vel"], 1);

    // Parameters setup
    std::vector<long> movements = this->get_parameter("movements").as_integer_array();
    m_values = this->get_parameter("values").as_double_array();
    m_max_vel =     this->get_parameter("max_vel_in_m").as_double();
    m_max_acc =     this->get_parameter("max_acc_in_m").as_double();
    m_max_rot_vel = this->get_parameter("max_rot_vel_in_deg").as_double(); /* deg/s */
    m_max_rot_acc = this->get_parameter("max_rot_acc_in_deg").as_double(); /* deg/s^2 */
    m_dt_in_ns = std::chrono::nanoseconds(this->get_parameter("update_period_in_ms").as_int() * 1000000ul);
    m_dt = m_dt_in_ns.count()/1.0e9;

    m_use_close_loop = this->get_parameter("use_close_loop").as_bool();

    if(movements.size() == 0)
    {
      RCLCPP_ERROR(get_logger(), "No movements specified. Ending...");
      return;
    }

    m_movements.resize(movements.size());
    std::transform(movements.begin(), movements.end(), m_movements.begin(),[this](const long d){
      return d == 0? MoveType::LINEAR : MoveType::ANGULAR;
    });
    if (m_movements.size() != m_values.size())
    {
      RCLCPP_FATAL(this->get_logger(), "Parameters movements and value size are different! Exiting...");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Start in ");
    rclcpp::sleep_for(1s);
    RCLCPP_INFO(this->get_logger(), "...III");
    rclcpp::sleep_for(1s);
    RCLCPP_INFO(this->get_logger(), "...II");
    rclcpp::sleep_for(1s);
    RCLCPP_INFO(this->get_logger(), "...I");
    rclcpp::sleep_for(1s);
    RCLCPP_INFO(this->get_logger(), "Starting!");

    this->on_main();
  }

  void on_main()
  {
    RCLCPP_DEBUG(get_logger(), "Entering motion");
    const double t_la = m_max_vel/m_max_acc;
    const double t_ra = m_max_rot_vel/m_max_rot_acc;
    for(size_t idx = 0; idx < m_movements.size(); ++idx)
    {
      RCLCPP_DEBUG_STREAM(get_logger(),
                          "Movement: " << (m_move_type == MoveType::LINEAR? "Linear" : "Angular") << "\n"<<
                          "Value: " << m_values.at(idx));
      m_move_type = m_movements.at(idx);
      double t_a, qp_max, qpp_max;
      if(m_move_type == MoveType::LINEAR)
      {
        t_a =     t_la;
        qp_max =  m_max_vel;
        qpp_max = m_max_acc;
      }
      else
      {
        t_a =     t_ra;
        qp_max =  m_max_rot_vel;
        qpp_max = m_max_rot_acc;
      }
      double duration = m_values.at(idx)/qp_max+t_a;
      double t = 0.0;

      double qp = 0.0, q = 0.0;
      geometry_msgs::msg::TwistStamped cmd_msg;
      cmd_msg.twist.linear.x = 0.0;
      cmd_msg.twist.linear.y = 0.0;
      cmd_msg.twist.angular.x = 0.0;
      cmd_msg.twist.angular.y = 0.0;

      rclcpp::Rate rate(m_dt_in_ns);

      RCLCPP_DEBUG(get_logger(),"Starting motion");
      std::chrono::time_point t_start = std::chrono::steady_clock::now();
      while(t < duration + duration/20)
      {
        rate.reset();
        if(t < t_a)
        {
          qp = qpp_max * t;
          q  = 0.5*qpp_max*std::pow(t,2);
        }
        else if(t_a <= t && t <= duration - t_a)
        {
          qp = qp_max;// qpp_max*t_a;
          q = qpp_max*t_a*(t - 0.5*t_a);
        }
        else if(t < duration)
        {
          qp = qpp_max * (duration - t);
          q = m_values.at(idx) - 0.5*qpp_max*std::pow((duration - t), 2);
        }
        else
        {
          qp = 0;
          q = m_values.at(idx);
        }

        t += m_dt;

//        qp /= qp_max;
        if(m_move_type == MoveType::LINEAR)
        {
          cmd_msg.twist.linear.x = qp;
          cmd_msg.twist.angular.z = 0.0;
        }
        else
        {
          cmd_msg.twist.linear.z = 0.0;
          cmd_msg.twist.angular.z = qp;
        }
        cmd_msg.header.stamp = this->get_clock()->now();
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
                         "Planned space to travel: " << m_values.at(idx)
                         );
    }
    RCLCPP_INFO(get_logger(), "All movements completed.");
  }

  bool set_home()
  {

  }

  bool go_home()
  {

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
  std::chrono::nanoseconds m_dt_in_ns;
  double m_dt;

  bool m_use_close_loop;

  std::unordered_map<std::string, std::string> m_topics;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_cmd_vel__pub;
  geometry_msgs::msg::PoseStamped m_pose_feedback;
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>();
  rclcpp::sleep_for(1s);
  RCLCPP_WARN(rclcpp::get_logger("motion_test"), "Shutting down");
  rclcpp::shutdown();
  return 0;
}

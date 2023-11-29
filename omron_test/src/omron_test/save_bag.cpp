#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/executors.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int16.hpp>

#include "rosbag2_cpp/writer.hpp"

#include <chrono>

using namespace std::chrono_literals;

class BagWriter : public rclcpp::Node
{
public:
  BagWriter() : rclcpp::Node("test_bag_writer"), m_test_number(0), m_is_bag_open(false)
  {
    RCLCPP_WARN(get_logger(), "Creating BagWriter");
    this->declare_parameter("cmd_vel_topic",       rclcpp::ParameterType::PARAMETER_STRING);
    this->declare_parameter("state_pose_topic",    rclcpp::ParameterType::PARAMETER_STRING);
    this->declare_parameter("state_vel_topic",     rclcpp::ParameterType::PARAMETER_STRING);
    this->declare_parameter("test_number_topic",   rclcpp::ParameterType::PARAMETER_STRING);
    this->declare_parameter("bag_path",            rclcpp::ParameterType::PARAMETER_STRING);

    m_topics["cmd_vel"] =    this->get_parameter("cmd_vel_topic").as_string();
    m_topics["state_pose"] = this->get_parameter("state_pose_topic").as_string();
    m_topics["state_vel"] =  this->get_parameter("state_vel_topic").as_string();
    m_topics["test_num"] =   this->get_parameter("test_number_topic").as_string();
    rclcpp::SubscriptionOptions options;
    options.callback_group = m_cb_group;

    using namespace std::placeholders;
    m_test_number__sub = this->create_subscription<std_msgs::msg::Int16>(m_topics["test_num"],
                                                                         5,
                                                                         std::bind(&BagWriter::manage_bag__cb, this, _1));
    m_cmd_twist__sub = this->create_subscription<geometry_msgs::msg::Twist>(m_topics["cmd_vel"],
                                                                             10,
                                                                             std::bind(&BagWriter::cmd_twist__cb, this, _1));
    m_state_pose__sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(m_topics["state_pose"],
                                                                             10,
                                                                             std::bind(&BagWriter::pose__cb, this, _1));
    m_state_twist__sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(m_topics["state_vel"],
                                                                               10,
                                                                               std::bind(&BagWriter::twist__cb, this,_1));
    m_path_bags =   this->get_parameter("bag_path").as_string();
    m_bag_writer = std::make_unique<rosbag2_cpp::Writer>();
    m_timestamp = this->get_clock()->now().nanoseconds();
//    m_bag_writer->open(m_path_bags + "/test_move_" + std::to_string(m_timestamp) + "__" + std::to_string(m_test_number));
//    m_bag_writer->create_topic({m_topics["cmd_vel"], "geometry_msgs/msg/Twist", rmw_get_serialization_format(), ""});
//    m_bag_writer->create_topic({m_topics["state_pose"], "geometry_msgs/msg/PoseStamped", rmw_get_serialization_format(), ""});
//    m_bag_writer->create_topic({m_topics["state_vel"], "geometry_msgs/msg/TwistStamped", rmw_get_serialization_format(), ""});
//    m_is_bag_open = true;
  }
  ~BagWriter()
  {
    RCLCPP_INFO(get_logger(), "Exiting");
    RCLCPP_WARN(get_logger(), "Received:\n\tpose_msgs = %d\n\tvel_msgs = %d\n\tcmd_vel = %d",
                m_pose_count__dbg,m_twist_count__dbg, m_cmd_twist_count__dbg);
    m_bag_writer->close();
  }
  int m_twist_count__dbg {0}, m_pose_count__dbg {0}, m_cmd_twist_count__dbg {0};

private:
  void pose__cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard guard(m_mtx2);
    if(m_is_bag_open)
    {
      ++m_pose_count__dbg;
      rclcpp::SerializedMessage serial_msg;
      m_serial_pose.serialize_message(msg.get(), &serial_msg);
      m_bag_writer->write(serial_msg, m_topics["state_pose"], "geometry_msgs/msg/PoseStamped", msg->header.stamp);
    }
  }
  void twist__cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    std::lock_guard guard(m_mtx3);
    if(m_is_bag_open)
    {
      ++m_twist_count__dbg;
      rclcpp::SerializedMessage serial_msg;
      m_serial_twist.serialize_message(msg.get(), &serial_msg);
      m_bag_writer->write(serial_msg, m_topics["state_vel"], "geometry_msgs/msg/TwistStamped", msg->header.stamp);
    }
  }
  void cmd_twist__cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard guard(m_mtx1);
    if(m_is_bag_open)
    {
      ++m_cmd_twist_count__dbg;
      rclcpp::SerializedMessage serial_msg;
      m_serial_twist.serialize_message(msg.get(), &serial_msg);
      m_bag_writer->write(serial_msg, m_topics["cmd_vel"], "geometry_msgs/msg/Twist", get_clock()->now());
    }
  }
  void manage_bag__cb(const std_msgs::msg::Int16& msg)
  {
    RCLCPP_WARN(get_logger(), "Changing bag status -> received: %d", msg.data);
    std::scoped_lock(m_mtx1, m_mtx2, m_mtx3);
    if(m_is_bag_open)
    {
      if(msg.data == 0)
      {
        RCLCPP_INFO(get_logger(), "Close bag");
        m_bag_writer->close();
        m_is_bag_open = false;
      }
      else if(msg.data != m_test_number)
      {
        RCLCPP_INFO(get_logger(), "Close bag");
        m_bag_writer->close();
        m_is_bag_open = false;
        m_test_number = msg.data;
        RCLCPP_INFO(get_logger(), "Open new bag");
        m_bag_writer = std::make_unique<rosbag2_cpp::Writer>();
        m_bag_writer->open(m_path_bags + "/test_move_" + std::to_string(m_timestamp) + "__" + std::to_string(m_test_number));
        m_bag_writer->create_topic({m_topics["cmd_vel"], "geometry_msgs/msg/Twist", rmw_get_serialization_format(), ""});
        m_bag_writer->create_topic({m_topics["state_pose"], "geometry_msgs/msg/PoseStamped", rmw_get_serialization_format(), ""});
        m_bag_writer->create_topic({m_topics["state_vel"], "geometry_msgs/msg/TwistStamped", rmw_get_serialization_format(), ""});
        m_is_bag_open = true;
      }
    }
    else
    {
      if (msg.data != 0)
      {
        RCLCPP_INFO(get_logger(), "Open new bag");
        m_test_number = msg.data;
        m_bag_writer->open(m_path_bags + "/test_move_" + std::to_string(m_timestamp) + "__" + std::to_string(m_test_number));
        m_bag_writer->create_topic({m_topics["cmd_vel"], "geometry_msgs/msg/Twist", rmw_get_serialization_format(), ""});
        m_bag_writer->create_topic({m_topics["state_pose"], "geometry_msgs/msg/PoseStamped", rmw_get_serialization_format(), ""});
        m_bag_writer->create_topic({m_topics["state_vel"], "geometry_msgs/msg/TwistStamped", rmw_get_serialization_format(), ""});
        m_is_bag_open = true;
      }
    }
  }

  int m_test_number;
  unsigned long long int m_timestamp;

  std::unordered_map<std::string, std::string> m_topics;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr m_test_number__sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_twist__sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_state_pose__sub;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr m_state_twist__sub;
  rclcpp::CallbackGroup::SharedPtr m_cb_group;
  std::string m_path_bags;
  std::unique_ptr<rosbag2_cpp::Writer> m_bag_writer;
  rclcpp::Serialization<geometry_msgs::msg::PoseStamped> m_serial_pose;
  rclcpp::Serialization<geometry_msgs::msg::TwistStamped> m_serial_twist;

  bool m_is_bag_open;
  std::mutex m_mtx1, m_mtx2, m_mtx3;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BagWriter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

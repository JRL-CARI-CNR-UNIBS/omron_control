#include <Aria/Aria.h>
#include <ArNetworking/ArNetworking.h>
#include <ArNetworking/ArClientHandlerRobotUpdate.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>

class JogNode : public rclcpp::Node
{
public:
  JogNode()
    : rclcpp::Node("JogNode"),
      get_pose_status__ftor(this, &JogNode::get_pose_status__cb)
  {
    using namespace std::placeholders;
    using namespace std::chrono_literals;
    cmd_jog__sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("jog/cmd", 10, std::bind(&JogNode::send_cmd__cb, this, _1));
    status_pose__pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("jog/pose",1);
    status_twist__pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("jog/twist",1);
    mode__pub = this->create_publisher<std_msgs::msg::String>("jog/mode",1);

    // Aria setup
    m_client.enforceProtocolVersion("5MTX");
    m_args.addPlain("-host");
    m_args.addPlain("192.168.1.32");
    m_args.addPlain("-p");
    m_args.addPlain("7272");
    m_args.addPlain("-u");
    m_args.addPlain("admin");
    m_args.addPlain("-pwd");
    m_args.addPlain("admin");
    m_client_connector = std::make_shared<ArClientSimpleConnector>(&m_args);
    m_client_connector->parseArgs();
    RCLCPP_INFO(this->get_logger(), "Connection parameter: OK");
    if (!m_client_connector->connectClient(&m_client))
    {
      if (m_client.wasRejected())
        RCLCPP_FATAL_STREAM(this->get_logger(), "Server" << m_client.getHost() << "rejected connection, exiting");
      else
        RCLCPP_FATAL_STREAM(this->get_logger(), "Could not connect to server" << m_client.getHost() <<  "exiting");
    }

    if(m_client.dataExists("updateNumbers"))
    {
      m_client_update = std::make_shared<ArClientHandlerRobotUpdate>(&m_client);
      m_client_update->requestUpdates();
      m_client_update->addUpdateCB(&get_pose_status__ftor);
    }
    else
    {
      RCLCPP_FATAL(this->get_logger(), "'updateNumbers' request cannot be set. Exiting...");
    }

    m_client.runAsync();

      rclcpp::sleep_for(500ms);
      m_client.logDataList();
  }

private:
  void send_cmd__cb(const std_msgs::msg::Float64MultiArray& msg)
  {
    if(msg.data.size() < 3)
    {
      RCLCPP_ERROR(get_logger(), "Multiarray requires at least 4 elements. %d given", msg.data.size());
      return;
    }
    switch((int) msg.data[0])
    {
      case 0: RCLCPP_INFO(get_logger(), "moveDistance: %f", msg.data[0]);
              m_client.requestOnceWithDouble("moveDistance", msg.data[0]);
              break;
      case 1: RCLCPP_INFO(get_logger(), "turnByAngle: %f", msg.data[1]);
              m_client.requestOnceWithDouble("turnByAngle", msg.data[1]);
              break;
      case 2: RCLCPP_INFO(get_logger(), "turnToHeading: %f", msg.data[2]);
              m_client.requestOnceWithDouble("turnToHeading", msg.data[2]);
              break;
      default: RCLCPP_ERROR(get_logger(), "Command invalid: %d", (int) msg.data[0]);
    }
  }

  void get_pose_status__cb(ArClientHandlerRobotUpdate::RobotData data)
  {
    double x,y,th,v_x,v_y,v_th,bat;
    std::string mode;
    mode = m_client_update->getMode();
    bat =  data.voltage;
    x =    data.pose.getX();
    y =    data.pose.getY();
    th =   data.pose.getThRad();
    v_x =  data.vel;
    v_y =  data.latVel;
    v_th = data.rotVel;

    geometry_msgs::msg::PoseStamped msg_pose;
    msg_pose.header.frame_id = "base_link";
    msg_pose.header.stamp = this->get_clock()->now();
    msg_pose.pose.position.x = x;
    msg_pose.pose.position.y = y;
    msg_pose.pose.position.z = 0;
    msg_pose.pose.orientation.x = 0;
    msg_pose.pose.orientation.y = 0;
    msg_pose.pose.orientation.z = std::sin(th/2.0);
    msg_pose.pose.orientation.w = std::cos(th/2.0);
    status_pose__pub->publish(msg_pose);

    geometry_msgs::msg::TwistStamped msg_twist;
    msg_twist.header.stamp = this->get_clock()->now();
    msg_twist.twist.linear.x = v_x;
    msg_twist.twist.linear.y = v_y;
    msg_twist.twist.linear.z = 0;
    msg_twist.twist.angular.x = 0;
    msg_twist.twist.angular.y = 0;
    msg_twist.twist.angular.z = v_th;
    status_twist__pub->publish(msg_twist);

    std_msgs::msg::String msg_mode;
    msg_mode.data = mode;
    mode__pub->publish(msg_mode);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr status_pose__pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr status_twist__pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode__pub;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_jog__sub;
  ArClientBase m_client;
  ArArgumentBuilder m_args;
  std::shared_ptr<ArClientSimpleConnector> m_client_connector;
  ArFunctor1C<JogNode, ArClientHandlerRobotUpdate::RobotData> get_pose_status__ftor;
  std::shared_ptr<ArClientHandlerRobotUpdate> m_client_update;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  Aria::init();

  rclcpp::spin(std::make_shared<JogNode>());
  rclcpp::shutdown();
  Aria::exit(0);
  return 0;
}

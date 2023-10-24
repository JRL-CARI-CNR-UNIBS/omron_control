#ifndef OMRON_HARDWARE_INTERFACE_HPP
#define OMRON_HARDWARE_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <Aria/Aria.h>
//#include <Aria/ArFunctor.h>
//#include <Aria/ArArgumentBuilder.h>
#include <ArNetworking/ArNetworking.h>
#include <ArNetworking/ArClientHandlerRobotUpdate.h>
//#include <ArNetworking/ArClientBase.h>
//#include <ArNetworking/ArClientSimpleConnector.h>

using namespace std::chrono_literals;
namespace omron {

class OmronAria : public hardware_interface::SystemInterface
{
public:
//  struct StatusData{
//    double battery_voltage, temperature;
//    struct Pose{
//      double x,y,rz;
//    } pose;
//    struct Twist{
//      double x,y,rz;
//    } vel;
//    struct Docking{
//      enum class DockState : int {
//        Undocked,
//        IsDocking,
//        Docked,
//        IsUndocking,
//        Unknown
//      } state;
//      int forced_dock{std::numeric_limits<int>::quiet_NaN()},
//          sec_until_shutdown{std::numeric_limits<int>::quiet_NaN()};
//    } dock;
//  };

  OmronAria();
  ~OmronAria(){}

  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) final;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) final;

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                            const std::vector<std::string>& stop_interfaces) final;
private:
  rclcpp::Logger m_logger = rclcpp::get_logger("OmronAria");

  int m_hz {100};

  struct ConnectionData {
    std::string ip;
    std::string port;
    std::string user;
    std::string pwd;
  } m_connection_data;

  std::vector<double> m_twist__command;
  std::vector<double> m_twist__states;
  std::vector<double> m_pose__states; // p.x,p.y,p.z,q.w,q.x,q.y,q.z

  const std::string HW_FORWARD_VEL = "forward";
  const std::string HW_TURN_VEL = "turn";
  const std::string GPIO_POSE = "sensor_pose";
  const std::vector<std::string> HW_POSE = {"pose.position.x",
                                            "pose.position.y",
                                            "pose.orientation.z"};
  enum class CommandType
  {
    Pose, // not implemented
    Twist
  };

  ArClientBase m_client;
  ArArgumentBuilder m_args;
  ArClientSimpleConnector m_client_connector;
  ArClientHandlerRobotUpdate m_client_update;

//  ArFunctor1C<OmronAria, ArNetPacket*> get_pose_status__ftor;
  ArFunctor1C<OmronAria, ArClientHandlerRobotUpdate::RobotData> get_pose_status__ftor;
  ArFunctor1C<OmronAria, ArNetPacket*> get_dock_status__ftor;

//  StatusData* m_status_data;
//  void get_pose_status__cb(ArNetPacket *packet);
//  void localise(); // TODO: da implementare con un'altra command_interface

  // TODO: implementa wall_time per limitare gli spostamenti
  void set_cmd_vel(const double& forward, const double& turn);
  bool m_is_cmd_valid;
};

} //omron


#endif // OMRON_HARDWARE_INTERFACE_HPP

#include "omron_hardware_interface/omron_map_client.hpp"
#include "omron_hardware_interface/omron_laser_client.hpp"
#include "omron_hardware_interface/omron_goto_goal_client.hpp"

#include <Aria/Aria.h>
#include <ArNetworking/ArNetworking.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#include <regex>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  Aria::init();

  if(not std::regex_match(argv[1], std::regex("^\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}$")))
  {
    std::cerr << "Argument is not an IP" << std::endl;
    exit(-2);
  }

  ArClientBase ar_client;
  ar_client.enforceProtocolVersion("5MTX");

  ArArgumentBuilder args;

  args.addPlain("-host");
  args.addPlain(argv[1]);  //Default IP

  //PORT
  args.addPlain("-p");
  args.addPlain("7272");  //Default PORT

  //USER
  args.addPlain("-u");
  args.addPlain("admin");  //Default user

  //PASSWD
  args.addPlain("-pwd admin");

  ArClientSimpleConnector clientConnector(&args);

  //Reard in args
  clientConnector.parseArgs();

  //Connect
  if (!clientConnector.connectClient(&ar_client))
  {
    if (ar_client.wasRejected())
      std::cout << "Server" << ar_client.getHost() << "rejected connection, exiting\n" << "\n";
    else
      std::cout << "Could not connect to server" << ar_client.getHost() <<  "exiting\n" << "\n";;
    exit(1);
  }

  ar_client.runAsync();
  // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  std::cout << "Connected to server.\n";
  rclcpp::executors::MultiThreadedExecutor exec;

  rclcpp::Node::SharedPtr map_node = std::make_shared<OmronMapClient>(&ar_client);
  rclcpp::Node::SharedPtr laser_node = std::make_shared<OmronLaserClient>(&ar_client, "Laser_1Current", "cloud_in");
  rclcpp::Node::SharedPtr goal_node = std::make_shared<OmronGotoGoalClient>(&ar_client);

  exec.add_node(map_node);
  exec.add_node(laser_node);
  exec.add_node(goal_node);

  exec.spin();

  rclcpp::shutdown();
  ar_client.disconnect();
  Aria::exit(0);
  return 0;
}

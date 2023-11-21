#ifndef OMRON_MAP_CLIENT_HPP
#define OMRON_MAP_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>

#include <Aria/Aria.h>
#include <ArNetworking/ArNetworking.h>

class OmronMapClient : public rclcpp::Node
{
public:
  OmronMapClient(const std::shared_ptr<ArClientBase> t_client);
  void init();
//  void run();
  static void init_and_run(const std::shared_ptr<ArClientBase> t_client);
private:
//  OmronMapClient(std::string node_name, ArClientBase *client);

  geometry_msgs::msg::PoseStamped currentPose;
  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr m_metadata__pub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map__pub;


//  ArClientBase *myClient;
  std::shared_ptr<ArClientBase> m_client;

  ArFunctor1C<OmronMapClient, ArNetPacket *> get_map_name__cb;
  ArFunctor1C<OmronMapClient, ArNetPacket *> get_map__cb;

  ArMap m_ar_map;
  ArTime m_start;


  void handle_get_map_name(ArNetPacket *packet);
  void handle_get_map(ArNetPacket *packet);

  int mapped = 0;

  nav_msgs::msg::MapMetaData    m_meta_data_message;
  nav_msgs::msg::OccupancyGrid  m_map_resp;


};

#endif // OMRON_MAP_CLIENT_HPP

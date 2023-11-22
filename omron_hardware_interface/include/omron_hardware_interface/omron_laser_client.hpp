#ifndef OMRON_LASER_CLIENT_HPP
#define OMRON_LASER_CLIENT_HPP

#include <Aria/Aria.h>
#include <ArNetworking/ArNetworking.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class OmronLaserClient: public rclcpp::Node
{
public:
  OmronLaserClient(ArClientBase *client, std::string laser_name="Laser_1Current", std::string topic="/laser");
  void laser__cb(ArNetPacket *packet);

protected:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_laser__pub;
  ArClientBase* m_client;
  ArFunctor1C<OmronLaserClient, ArNetPacket *> m_ar_laser__ftor;
};

#endif // OMRON_LASER_CLIENT_HPP

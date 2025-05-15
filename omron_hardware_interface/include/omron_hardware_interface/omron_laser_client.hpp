#ifndef OMRON_LASER_CLIENT_HPP
#define OMRON_LASER_CLIENT_HPP

#include <Aria/Aria.h>
#include <ArNetworking/ArNetworking.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include <mutex>


class OmronLaserClient: public rclcpp::Node
{
public:
  OmronLaserClient(ArClientBase *client, std::string laser_name="Laser_1Current", std::string topic="/laser");
  void laser__cb(ArNetPacket *packet);
  void pose__cb(ArNetPacket *packet);

protected:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_laser__pub;
  ArClientBase* m_client;
  ArFunctor1C<OmronLaserClient, ArNetPacket *> m_ar_laser__ftor;
  ArFunctor1C<OmronLaserClient, ArNetPacket *> m_ar_pose__ftor;
  std::string m_pose_handler_name;
  Eigen::Isometry2d m_T_map_base, m_T_base_map;
  std::string m_base_frame;
  std::mutex m_tf_mutex;
};

#endif // OMRON_LASER_CLIENT_HPP

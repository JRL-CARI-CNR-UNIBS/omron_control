#include "omron_hardware_interface/omron_laser_client.hpp"

OmronLaserClient::OmronLaserClient(ArClientBase *client, std::string laser_name, std::string topic):
  Node("omron_laser_client"),
  m_client(client),
  m_ar_laser__ftor(this, &OmronLaserClient::laser__cb),
  m_ar_pose__ftor(this, &OmronLaserClient::pose__cb),
  m_pose_handler_name("updateNumbers")
{
  this->declare_parameter("base_footprint_frame",rclcpp::ParameterValue("base_footprint"));
  m_base_frame = this->get_parameter("base_footprint_frame").as_string();

  m_laser__pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, rclcpp::SensorDataQoS());
  m_client->addHandler(laser_name.c_str(), &m_ar_laser__ftor);
  m_client->addHandler(m_pose_handler_name.c_str(), &m_ar_pose__ftor);
  m_client->request(laser_name.c_str(), 100);
  m_client->request(m_pose_handler_name.c_str(), 100);

  RCLCPP_WARN(this->get_logger(),"Setup Callback for %s publishing on %s",laser_name.c_str(), topic.c_str());
}

void OmronLaserClient::pose__cb(ArNetPacket *packet)
{
  packet->bufToByte2(); // Discard battery voltage
  m_T_map_base.translation()(0) =            (  (double) packet->bufToByte4() )/1000.0;
  m_T_map_base.translation()(1) =            (  (double) packet->bufToByte4() )/1000.0;
//  Eigen::AngleAxisd aax((double) packet->bufToByte2(), Eigen::Vector3d::UnitZ()); // float?
  Eigen::Rotation2D rot((double) (packet->bufToByte2()) * M_PI/180.0);
  m_T_map_base.linear() = rot.toRotationMatrix();
}

void OmronLaserClient::laser__cb(ArNetPacket *packet)
{
  sensor_msgs::msg::PointCloud2 actual_scan;

  actual_scan.header.stamp = this->get_clock()->now();
  // actual_scan.header.m_seq = m_seq++;
  actual_scan.header.frame_id = m_base_frame;

  int numReadings;
  int i;

  numReadings = packet->bufToByte4();

  actual_scan.height = 1;
  actual_scan.width  = numReadings;
  actual_scan.fields.resize (3);
  actual_scan.fields[0].name = "x";
  actual_scan.fields[0].offset = 0;
  actual_scan.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  actual_scan.fields[0].count = 1;
  actual_scan.fields[1].name = "y";
  actual_scan.fields[1].offset = 4;
  actual_scan.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  actual_scan.fields[1].count = 1;
  actual_scan.fields[2].name = "z";
  actual_scan.fields[2].offset = 8;
  actual_scan.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  actual_scan.fields[2].count = 1;
  actual_scan.point_step = 12;
  actual_scan.row_step   = actual_scan.point_step * actual_scan.width;
  actual_scan.data.resize (actual_scan.row_step   * actual_scan.height);
  actual_scan.is_dense = false;

  if (numReadings == 0)
  {
    RCLCPP_WARN(this->get_logger(),"No readings for sensor %s\n\n", m_client->getName(packet));
    return;
  }

  Eigen::Affine2d T_map_scan, T_base_scan;
  for (i = 0; i < numReadings; i++)
  {
    T_map_scan.translation()(0) = (double)packet->bufToByte4();
    T_map_scan.translation()(1) = (double)packet->bufToByte4();

    float *pstep = (float*)&actual_scan.data[i * actual_scan.point_step];

    T_base_scan = m_T_map_base.inverse() * T_map_scan;

    pstep[0] = T_base_scan.translation()(0)/1000.0;
    pstep[1] = T_base_scan.translation()(1)/1000.0;
    pstep[2] = 0.3; // Height of the lidar
  }

  m_laser__pub->publish(actual_scan);
}

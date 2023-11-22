#include "omron_hardware_interface/omron_laser_client.hpp"

OmronLaserClient::OmronLaserClient(ArClientBase *client, std::string laser_name, std::string topic):
  Node("omron_laser_client"),
  m_client(client),
  m_ar_laser__ftor(this, &OmronLaserClient::laser__cb)
{
  m_laser__pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, rclcpp::SensorDataQoS());
  m_client->addHandler(laser_name.c_str(), &m_ar_laser__ftor);
  m_client->request(laser_name.c_str(), 100);

  RCLCPP_WARN(this->get_logger(),"Setup Callback for %s publishing on %s",laser_name.c_str(), topic.c_str());
}

void OmronLaserClient::laser__cb(ArNetPacket *packet)
{
  sensor_msgs::msg::PointCloud2 actual_scan;

  actual_scan.header.stamp = this->get_clock()->now();
  // actual_scan.header.m_seq = m_seq++;
  actual_scan.header.frame_id = "map";

  int x, y;
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

  for (i = 0; i < numReadings; i++)
  {
    x = packet->bufToByte4();
    y = packet->bufToByte4();

    float *pstep = (float*)&actual_scan.data[i * actual_scan.point_step];

    pstep[0] = x/1000.0;
    pstep[1] = y/1000.0;
    pstep[2] = 0;
  }

  m_laser__pub->publish(actual_scan);
}

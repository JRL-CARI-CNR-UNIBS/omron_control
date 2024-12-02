#include <omron_hardware_interface/omron_map_client.hpp>

OmronMapClient::OmronMapClient(ArClientBase* t_client)
  : rclcpp::Node("omron_map_client"),
    m_get_map_name__ftor(this, &OmronMapClient::handle_get_map_name),
    m_get_map__ftor(this, &OmronMapClient::handle_get_map),
    mapped(0)
{
  m_client = t_client;
  init();
}

void OmronMapClient::init()
{
  m_metadata__pub = this->create_publisher<nav_msgs::msg::MapMetaData>("map_metadata",rclcpp::QoS(2).reliable().transient_local());
  m_map__pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map",rclcpp::QoS(2).reliable().transient_local());

  m_client->addHandler("getMap", &m_get_map__ftor);
  m_client->addHandler("getMapName", &m_get_map_name__ftor);
  m_client->requestOnce("getMapName");
  m_start.setToNow();
  m_client->requestOnce("getMap");

  while(!mapped){
    RCLCPP_INFO(this->get_logger(), "Waiting for map");
    sleep(1);
  }

  m_metadata__pub->publish(m_meta_data_message);
  m_map__pub->publish(m_map_resp);
  RCLCPP_INFO(this->get_logger(), "Map sent");
}

void OmronMapClient::handle_get_map_name(ArNetPacket *packet)
{
  char buffer[512];

  packet->bufToStr(buffer, sizeof(buffer));
  printf("MapFile: %s\n", buffer);
}

void OmronMapClient::handle_get_map(ArNetPacket *packet)
{
  char buffer[10000];

  if (packet->getDataReadLength() == packet->getDataLength())
  {
    printf("Empty packet signifying end of map (for central forward)\n");
    return;
  }

  packet->bufToStr(buffer, sizeof(buffer));
  // if we got an end of line char instead of a line it means the map is over
  if (buffer[0] == '\0')
  {
    printf("First line \n %.*s", 20 , buffer);

    printf("Map took %g seconds\n", m_start.mSecSince() / 1000.0);
    m_ar_map.parsingComplete();

    int n_points = m_ar_map.getNumPoints();
    printf("Map has %d points\n", n_points);

    std::vector<ArPose> *point_list = new std::vector<ArPose> (n_points);
    point_list = m_ar_map.getPoints();

    int res = m_ar_map.getResolution();
    printf("Map has resolution of %dmm\n", res);

    ArPose minPose, maxPose;
    minPose = m_ar_map.getMinPose();
    maxPose = m_ar_map.getMaxPose();

    printf("Map has Min Coords of (%f %f) mm\n", minPose.getX(), minPose.getY() );
    printf("Map has Min Coords of (%f %f) mm\n", maxPose.getX(), maxPose.getY() );

    int gridX = (maxPose.getX() - minPose.getX())/res;
    int gridY = (maxPose.getY() - minPose.getY())/res;

    printf("Map has grid of (%d %d) mm\n", gridX, gridY);


    //client.disconnect();
    //exit(0);

    m_map_resp.info.width = gridX;
    m_map_resp.info.height = gridY;
    m_map_resp.info.resolution = res/1000.0;

    m_map_resp.info.origin.position.x = minPose.getX()/1000.0; //Y?
    m_map_resp.info.origin.position.y = minPose.getY()/1000.0; //X?

    m_map_resp.info.map_load_time = this->get_clock()->now();
    m_map_resp.header.frame_id = "map";
    m_map_resp.header.stamp = this->get_clock()->now();
    RCLCPP_WARN(this->get_logger(),"Read a %d X %d map @ %.3lf m/cell",
            m_map_resp.info.width,
            m_map_resp.info.height,
            m_map_resp.info.resolution);
    m_meta_data_message = m_map_resp.info;

    //Lets fill some datas
    m_map_resp.data.resize(gridX * gridY);

    //Iterate through points and fill
    for(int i=0; i < point_list->size(); i++){
      int coord = gridX *  ((point_list->at(i).getY()-minPose.getY())/res) + (point_list->at(i).getX()-minPose.getX()) /res; //Y is flipped apparent
      //printf("Point at Coord %d of %d\n", coord, gridX*gridY);
      m_map_resp.data[coord] = 100;
    }
    printf("Map retrieved");
    mapped = 1;

  }
  else
  {

    //The header has changed but it still works with the old format for what we want.
    char *header_location = strstr(buffer, "2D-Map-Ex4");
    if (header_location != NULL) /* Old header found */
    {
      buffer[6] = '\0'; //Cut the -EX4 off
    }


    //printf("line '%s'\n", buffer);
    m_ar_map.parseLine(buffer);
  }

  m_metadata__pub->publish(m_meta_data_message);
  m_map__pub->publish(m_map_resp);
}

//void OmronMapClient::init_and_run(const std::shared_ptr<ArClientBase> t_client)
//{
//  rclcpp::Node::SharedPtr map_node = std::make_shared<OmronMapClient>(t_client);
//  rclcpp::spin(map_node);
//}

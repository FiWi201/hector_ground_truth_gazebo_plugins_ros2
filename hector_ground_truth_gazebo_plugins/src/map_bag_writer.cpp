#include <iostream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <map_bag/map_bag.hpp>
#include <map_bag_msgs/msg/map.hpp>
#include <map_bag_ros/message_conversions/map.hpp>
#include <map_bag/edit/map_bag_writer.hpp>
#include <map_bag/io.hpp>


#define SUBSCRIPTION_TOPIC "ground_truth/elevation_map"


class MapBagWriter : public rclcpp::Node
{
private:
  rclcpp::Subscription<map_bag_msgs::msg::Map>::SharedPtr grid_map_subscription_;

public:
  MapBagWriter() : Node("grid_map_writer")
  {
    declare_parameter("write_path", "../Environments/MapBags");
    declare_parameter("file_prefix", "ground_truth");

    grid_map_subscription_  = create_subscription<map_bag_msgs::msg::Map>(
      SUBSCRIPTION_TOPIC, 10,
      std::bind(&MapBagWriter::MapBagSubscriptionCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Listening on topic \"%s\"", SUBSCRIPTION_TOPIC);
  }
private:
  void MapBagSubscriptionCallback(map_bag_msgs::msg::Map msg)
  {
    RCLCPP_INFO(get_logger(), "Received map message");

    auto map_bag = std::make_shared<map_bag::MapBag>(msg.resolution);
    map_bag::Map::SharedPtr map = map_bag_ros::message_conversions::msgToMap(msg);

    if (map->frame() != map_bag->frame())
      map->_setFrame(map_bag->frame());
    
    map_bag->lockForWrite()->addMap(map);

    std::ofstream map_stream( "../Environments/MapBags/test.whm" );
    map_bag::io::writeToStream( *map_bag, map_stream );
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MapBagWriter>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_GROUND_TRUTH_GAZEBO_PLUGINS_ELEVATION_MAP_GROUND_TRUTH_PLUGIN_HPP
#define HECTOR_GROUND_TRUTH_GAZEBO_PLUGINS_ELEVATION_MAP_GROUND_TRUTH_PLUGIN_HPP

#include <hector_ground_truth_gazebo_plugins_msgs/srv/generate_ground_truth.hpp>
#include <rclcpp/rclcpp.hpp>
#include <map_bag_msgs/msg/map.hpp>
#include <gz/plugin/Register.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sim/System.hh>

using namespace gz;
using namespace sim;
using namespace systems;
using namespace rendering;

namespace hector_ground_truth_gazebo_plugins
{

class ElevationMapGroundTruthPlugin :
    public System,
    public ISystemConfigure,
    public ISystemPostUpdate
{
private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Service<hector_ground_truth_gazebo_plugins_msgs::srv::GenerateGroundTruth>::SharedPtr service_server_;
  rclcpp::Publisher<map_bag_msgs::msg::Map>::SharedPtr publisher_;
  ScenePtr scene_;

public:
  ElevationMapGroundTruthPlugin();

  ~ElevationMapGroundTruthPlugin() override;

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &_eventMgr) override;

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

private:
  bool GenerateGroundTruthCallback( hector_ground_truth_gazebo_plugins_msgs::srv::GenerateGroundTruth::Request::SharedPtr req,
                                    hector_ground_truth_gazebo_plugins_msgs::srv::GenerateGroundTruth::Response::SharedPtr resp );
};
}

GZ_ADD_PLUGIN(
  hector_ground_truth_gazebo_plugins::ElevationMapGroundTruthPlugin,
  gz::sim::System,
  hector_ground_truth_gazebo_plugins::ElevationMapGroundTruthPlugin::ISystemConfigure,
  hector_ground_truth_gazebo_plugins::ElevationMapGroundTruthPlugin::ISystemPostUpdate)

#endif

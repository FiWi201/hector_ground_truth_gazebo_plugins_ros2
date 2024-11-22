// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <Eigen/Core>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/RayQuery.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderEngine.hh>

#include <rclcpp/rclcpp.hpp>
#include <map_bag_msgs/msg/map.hpp>
#include <map_bag/map.hpp>
#include <map_bag_ros/message_conversions/map.hpp>
#include <hector_ground_truth_gazebo_plugins_msgs/srv/generate_ground_truth.hpp>
#include <hector_ground_truth_gazebo_plugins/elevation_map_ground_truth_plugin.hpp>


using namespace hector_ground_truth_gazebo_plugins_msgs;

using namespace hector_ground_truth_gazebo_plugins;

ElevationMapGroundTruthPlugin::ElevationMapGroundTruthPlugin()= default;

ElevationMapGroundTruthPlugin::~ElevationMapGroundTruthPlugin()= default;

void ElevationMapGroundTruthPlugin::Configure(const Entity &_entity,
                                              const std::shared_ptr<const sdf::Element> &_sdf,
                                              EntityComponentManager &_ecm,
                                              EventManager &_eventMgr)
{
  (void) _entity;
  (void) _sdf;
  (void) _ecm;
  (void) _eventMgr;

  // Initialize ROS node
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);
  node_ = std::make_shared<rclcpp::Node>("gazebo_ground_truth_node");

  // Start server and publisher
  using namespace std::placeholders;
  service_server_ = node_->create_service<hector_ground_truth_gazebo_plugins_msgs::srv::GenerateGroundTruth>(
    "ground_truth/generate_elevation_map",
    std::bind(&ElevationMapGroundTruthPlugin::GenerateGroundTruthCallback, this, _1, _2));
  publisher_ = node_->create_publisher<map_bag_msgs::msg::Map>(
    "ground_truth/elevation_map", 10);

  RCLCPP_INFO(node_->get_logger(), "Plugin was started");
}

bool ElevationMapGroundTruthPlugin::GenerateGroundTruthCallback( std::shared_ptr<hector_ground_truth_gazebo_plugins_msgs::srv::GenerateGroundTruth::Request> req,
                                                                 std::shared_ptr<hector_ground_truth_gazebo_plugins_msgs::srv::GenerateGroundTruth::Response> resp )
{
  RCLCPP_INFO(node_->get_logger(), "Request received");

  auto start_time = node_->get_clock()->now();

  // Get ray query for ray intersections
  scene_ = sceneFromFirstRenderEngine();
  if (scene_ == nullptr)
  {
    RCLCPP_ERROR(node_->get_logger(), "No rendering scene could be found!");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Rendering scene %s found", scene_->Name().c_str());

  const RayQueryPtr ray_query = scene_->CreateRayQuery();

  const long cells_x = std::ceil(req->x_size / req->resolution);
  const double cells_x_2 = cells_x / 2.0;
  const long cells_y = std::ceil(req->y_size / req->resolution);
  const double cells_y_2 = cells_y / 2.0;
  const double min_elevation = req->max_z_height - req->truncation_distance;
  long samples = req->samples;
  if ( samples <= 0 )
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Samples needs to be greater than 0. Set to 1, was: " << samples);
    samples = 1;
  }
  const double sample_dist = 1.0 / (samples + 1);
  hector_math::GridMapf elevation_map{cells_x, cells_y};

  const math::Quaterniond q( req->origin.orientation.w, req->origin.orientation.x, req->origin.orientation.y,
                       req->origin.orientation.z );
  std::string entity_name;
  const double resolution = req->resolution;
  const math::Vector3d origin( req->origin.position.x, req->origin.position.y, req->origin.position.z );

  constexpr float progress_step_size = 0.05f;
  float next_progress_step = progress_step_size;

  for ( Eigen::Index x = 0; x < cells_x; ++x )
  {
    if ((float) x / cells_x >= next_progress_step)
    {
      RCLCPP_INFO(node_->get_logger(), "Heightmap Progress: %3.0f%%", next_progress_step * 100);
      next_progress_step += progress_step_size;
    }
    for ( Eigen::Index y = 0; y < cells_y; ++y )
    {
      double min_dist = std::numeric_limits<double>::quiet_NaN();
      // Make samples^2 samples per cell
      for ( long xi = 0; xi < samples; ++xi )
      {
        for ( long yi = 0; yi < samples; ++yi )
        {
          math::Vector3d start((x - cells_x_2 + 0.5 + (xi - (samples - 1) / 2.0) * sample_dist) * resolution,
                               (y - cells_y_2 + 0.5 + (yi - (samples - 1) / 2.0) * sample_dist) * resolution,
                               req->max_z_height );

          ray_query->SetOrigin(origin + q * start);
          ray_query->SetDirection( {0, 0, -1} );

          const RayQueryResult ray_query_result = ray_query->ClosestPoint();
          if (ray_query_result)
            min_dist = ray_query_result.distance;
        }
      }

      float elevation = static_cast<float>(req->max_z_height - min_dist);

      // TODO Zugriff Optimieren
      // Matrix needs to be flipped as apparently grid map coordinate system is opposite to the gazebo coordinate system
      if (elevation > req->max_z_height || elevation < min_elevation)
        elevation = map_bag::UnknownValue;

      elevation_map( cells_x - 1 - x, cells_y - 1 - y ) = elevation;
    }
  }
  /* TODO The origin vector could be transformed to allow for multiple maps with the same
     Rotation to easily be joined in the same MapBag */
  auto result = std::make_shared<const map_bag::Map>(elevation_map, Eigen::Vector3d::Zero(), req->resolution);

  resp->map = map_bag_ros::message_conversions::mapToMapMsg(result, {"elevation"});;
  resp->map.header.frame_id = map_bag::Map::DEFAULT_FRAME;
  resp->map.header.stamp = node_->get_clock()->now();

  auto duration = node_->get_clock()->now().seconds() - start_time.seconds();
  RCLCPP_INFO( node_->get_logger(), "Height map computed in %.2f Seconds", duration );

  publisher_->publish( resp->map );
  return true;
}

void ElevationMapGroundTruthPlugin::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm)
{
  (void) _info;
  (void) _ecm;

  rclcpp::spin_some(node_);
}

#include "augment_gridmap_online/AugmentedGridMap.hpp"
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <algorithm>
#include <numeric>
#include <vector>

namespace ros_augmented_gridmaps {

AugmentedGridMap::AugmentedGridMap(ros::NodeHandle &nodeHandle)
  : nodeHandle_(nodeHandle)
{
  ros::NodeHandle private_nh("~");
  // Load parameters
  private_nh.param<float>("obstacle_radius", obstacle_radius, 0.05);
  private_nh.param<bool>("debug", debug, false);
  private_nh.param<std::string>("input_map", input_map, "map");
  private_nh.param<std::string>("env_file_name", fn_, "env_furniture_rc24_3330_2");
  private_nh.param<bool>("add_static_obstacles", is_adding_, true);

  // Initialize subscribers
  mapSubscriber = nodeHandle_.subscribe(input_map, 1, &AugmentedGridMap::saveMap, this);
  pointSubscriber = nodeHandle_.subscribe("point_obstacle", 1, &AugmentedGridMap::addPointCallback, this);

  // Initialize publishers
  augmented_map_pub = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("/grid_map/augmented_map", 1, true);
  augmented_metadata_pub = nodeHandle_.advertise<nav_msgs::MapMetaData>("/grid_map/augmented_map_metadata", 1, true);
  obstacle_marker_pub = nodeHandle_.advertise<visualization_msgs::Marker>("/grid_map/obstacle_markers", 1, true);

  // Initialize services
  clearServer = nodeHandle_.advertiseService("/clear_map", &AugmentedGridMap::clearMapCallback, this);
  getAugmentedMap = nodeHandle_.advertiseService("/grid_map/get_augmented_map", &AugmentedGridMap::getAugmentedMapCallback, this);

  ROS_INFO("Map enhancer node Initialization finished");
}

AugmentedGridMap::~AugmentedGridMap() {}

void AugmentedGridMap::saveMap(const nav_msgs::OccupancyGrid &map)
{
  original_map = map;
  enhanced_map = map;
  map_metadata = map.info;

  ROS_INFO("Got a map of: [%d, %d] @ %f resolution", enhanced_map.info.width, enhanced_map.info.height, enhanced_map.info.resolution);

  publishEnhancedMap();

  if (is_adding_) {
    ROS_INFO("STARTING DRAWING OBSTACLES");
    drawObstacles();
  }
}

void AugmentedGridMap::addPointCallback(const geometry_msgs::PointStamped &added_point)
{
  if (original_map.data.empty()) {
    ROS_ERROR("Do not have original map to enhance yet");
    return;
  }

  if (added_point.header.frame_id != original_map.header.frame_id) {
    ROS_ERROR("Point and map are on different frames!!!");
    ROS_ERROR("Point frame: %s", added_point.header.frame_id.c_str());
    ROS_ERROR("Map frame: %s", original_map.header.frame_id.c_str());
    return;
  }

  ROS_INFO("Adding point on: [%f, %f]", added_point.point.x, added_point.point.y);

  ClickedAddObstacleToMap(added_point);
  YamlAddObstacleToMap(added_point);

  publishEnhancedMap();
  makeClickedObstaclesMarkers();
}

void AugmentedGridMap::ClickedAddObstacleToMap(const geometry_msgs::PointStamped &added_point)
{
  float x_coord = added_point.point.x;
  float y_coord = added_point.point.y;

  float x_orig = map_metadata.origin.position.x;
  float y_orig = map_metadata.origin.position.y;

  float resolution = map_metadata.resolution;

  int cell_x = (x_coord - x_orig) / resolution;
  int cell_y = (y_coord - y_orig) / resolution;

  if (cell_x < 0 || cell_x >= static_cast<int>(map_metadata.width) || cell_y < 0 || cell_y >= static_cast<int>(map_metadata.height)) {
    ROS_ERROR("Point falls outside map");
    return;
  }

  obstacles.push_back(added_point.point);

  float min_x = std::max(0.0f, cell_x - obstacle_radius / resolution);
  float max_x = std::min(static_cast<float>(map_metadata.width), cell_x + obstacle_radius / resolution);
  float min_y = std::max(0.0f, cell_y - obstacle_radius / resolution);
  float max_y = std::min(static_cast<float>(map_metadata.height), cell_y + obstacle_radius / resolution);

  if (debug) {
    ROS_INFO("Point cell: [%d, %d]", cell_x, cell_y);
    ROS_INFO("Obstacle bounds - Min: [%f, %f], Max: [%f, %f]", min_x, min_y, max_x, max_y);
  }

  for (int i = static_cast<int>(min_x); i < static_cast<int>(max_x); ++i) {
    for (int j = static_cast<int>(min_y); j < static_cast<int>(max_y); ++j) {
      enhanced_map.data[i + j * map_metadata.width] = 100;
    }
  }

  makeClickedObstaclesMarkers();
  publishEnhancedMap();
  publishObstacleMarkers();
}

void AugmentedGridMap::YamlAddObstacleToMap(const geometry_msgs::PointStamped &added_point)
{
  int cell_x = pointToCell(added_point.point.x, map_metadata.origin.position.x, map_metadata.resolution, map_metadata.width);
  int cell_y = pointToCell(added_point.point.y, map_metadata.origin.position.y, map_metadata.resolution, map_metadata.height);

  if (cell_x < 0 || cell_x >= static_cast<int>(map_metadata.width) || cell_y < 0 || cell_y >= static_cast<int>(map_metadata.height)) {
    ROS_ERROR("Point falls outside map");
    return;
  }

  obstacles.push_back(added_point.point);

  int min_x = std::max(0, cell_x - static_cast<int>(obstacle_radius / map_metadata.resolution));
  int max_x = std::min(static_cast<int>(map_metadata.width), cell_x + static_cast<int>(obstacle_radius / map_metadata.resolution));
  int min_y = std::max(0, cell_y - static_cast<int>(obstacle_radius / map_metadata.resolution));
  int max_y = std::min(static_cast<int>(map_metadata.height), cell_y + static_cast<int>(obstacle_radius / map_metadata.resolution));

  if (debug) {
    ROS_INFO("Point cell: [%d, %d]", cell_x, cell_y);
    ROS_INFO("Obstacle bounds - Min: [%d, %d], Max: [%d, %d]", min_x, min_y, max_x, max_y);
  }

  for (int i = min_x; i < max_x; ++i) {
    for (int j = min_y; j < max_y; ++j) {
      enhanced_map.data[i + j * map_metadata.width] = 100;
    }
  }

  makeYamlObstaclesMarkers(min_x, max_x, min_y, max_y, (min_x + max_x) / 2, (min_y + max_y) / 2);
}

bool AugmentedGridMap::clearMapCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  enhanced_map = original_map;
  obstacle_markers_.clear();

  publishEnhancedMap();

  visualization_msgs::Marker marker_delete;
  marker_delete.action = visualization_msgs::Marker::DELETEALL;
  obstacle_marker_pub.publish(marker_delete);

  ROS_INFO("Restoring original map");
  return true;
}

bool AugmentedGridMap::getAugmentedMapCallback(nav_msgs::GetMap::Request& request, nav_msgs::GetMap::Response& response)
{
  response.map = enhanced_map;

  publishEnhancedMap();

  ROS_INFO("Getting augmented map");
  return true;
}

void AugmentedGridMap::makeClickedObstaclesMarkers()
{
  visualization_msgs::Marker obstacles_markers;
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 0;
  color.b = 0;
  color.a = 1.0;

  obstacles_markers.ns = "obstacles";
  obstacles_markers.action = visualization_msgs::Marker::ADD;
  obstacles_markers.header.frame_id = enhanced_map.header.frame_id;
  obstacles_markers.header.stamp = ros::Time();
  obstacles_markers.type = visualization_msgs::Marker::SPHERE_LIST;
  obstacles_markers.pose.orientation.w = 1.0;
  obstacles_markers.scale.x = obstacle_radius;
  obstacles_markers.scale.y = obstacle_radius;
  obstacles_markers.scale.z = obstacle_radius;
  obstacles_markers.id = 0;
  obstacles_markers.color = color;

  if (obstacles.empty()) {
    ROS_WARN("No obstacles to publish");
    return;
  }

  obstacles_markers.points = obstacles;

  ROS_INFO("Publishing clicked obstacle markers");
  obstacle_marker_pub.publish(obstacles_markers);
}

void AugmentedGridMap::makeYamlObstaclesMarkers(float min_x, float max_x, float min_y, float max_y, float cen_x, float cen_y)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = enhanced_map.header.frame_id.empty() ? "map" : enhanced_map.header.frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "obstacles";
  marker.id = obstacle_markers_.size();
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = cen_x;
  marker.pose.position.y = cen_y;
  marker.pose.position.z = 0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = max_x - min_x;
  marker.scale.y = max_y - min_y;
  marker.scale.z = 0.5;
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();

  obstacle_markers_.push_back(marker);
}

int AugmentedGridMap::pointToCell(float coordinate, float origin, float resolution, int max_index)
{
  return std::min(std::max(static_cast<int>((coordinate - origin) / resolution), 0), max_index);
}

void AugmentedGridMap::drawObstacles()
{
  std::string yaml_file = ros::package::getPath("hma_env_manage") + "/io/config/" + fn_;

  try {
    YAML::Node config = YAML::LoadFile(yaml_file);
    if (config["furniture"]) {
      for (const auto& it : config["furniture"]) {
        std::vector<float> x, y;
        for (const auto& data : it.second["plane"]) {
          x.push_back(data[0].as<float>());
          y.push_back(data[1].as<float>());
        }

        if (x.empty() || y.empty()) {
          ROS_ERROR("Empty list detected. Exiting.");
          return;
        }

        float min_x = *std::min_element(x.begin(), x.end());
        float max_x = *std::max_element(x.begin(), x.end());
        float min_y = *std::min_element(y.begin(), y.end());
        float max_y = *std::max_element(y.begin(), y.end());

        for (int i = pointToCell(min_x, map_metadata.origin.position.x, map_metadata.resolution, map_metadata.width); i < pointToCell(max_x, map_metadata.origin.position.x, map_metadata.resolution, map_metadata.width); ++i) {
          for (int j = pointToCell(min_y, map_metadata.origin.position.y, map_metadata.resolution, map_metadata.height); j < pointToCell(max_y, map_metadata.origin.position.y, map_metadata.resolution, map_metadata.height); ++j) {
            enhanced_map.data[i + j * map_metadata.width] = 100;
          }
        }

        float cen_x = std::accumulate(x.begin(), x.end(), 0.0f) / x.size();
        float cen_y = std::accumulate(y.begin(), y.end(), 0.0f) / y.size();

        makeYamlObstaclesMarkers(min_x, max_x, min_y, max_y, cen_x, cen_y);
      }

      publishEnhancedMap();
      publishObstacleMarkers();
    }
  } catch (const YAML::BadFile& e) {
    ROS_ERROR("Failed to load YAML file: %s", yaml_file.c_str());
  }
}

void AugmentedGridMap::publishEnhancedMap()
{
  augmented_map_pub.publish(enhanced_map);
  augmented_metadata_pub.publish(map_metadata);
}

void AugmentedGridMap::publishObstacleMarkers()
{
  for (const auto& marker : obstacle_markers_) {
    obstacle_marker_pub.publish(marker);
    ROS_INFO("Publishing markers");
  }
}

} // namespace ros_augmented_gridmaps


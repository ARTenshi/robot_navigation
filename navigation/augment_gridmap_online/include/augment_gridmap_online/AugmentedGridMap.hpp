#pragma once
// ROS
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/GetMap.h"
#include "visualization_msgs/Marker.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm> // for std::min and std::max

namespace ros_augmented_gridmaps {

class AugmentedGridMap
{
public:
      AugmentedGridMap(ros::NodeHandle& nodeHandle);
      ~AugmentedGridMap();

private:
      // ROS stuff
      ros::NodeHandle& nodeHandle_;
      ros::Subscriber mapSubscriber;
      ros::Subscriber pointSubscriber;
      ros::Publisher augmented_map_pub;
      ros::Publisher augmented_metadata_pub;
      ros::Publisher obstacle_marker_pub;
      ros::Subscriber obstacleSubscriber;
      ros::ServiceServer clearServer;
      ros::ServiceServer getAugmentedMap;

      void saveMap(const nav_msgs::OccupancyGrid &map);   
      void addPointCallback(const geometry_msgs::PointStamped &punto);   
      bool clearMapCallback(std_srvs::Empty::Request& request,
            std_srvs::Empty::Response& response);
      bool getAugmentedMapCallback(nav_msgs::GetMap::Request& request,
            nav_msgs::GetMap::Response& response);
      void ClickedAddObstacleToMap(const geometry_msgs::PointStamped& added_point);
      void YamlAddObstacleToMap(const geometry_msgs::PointStamped& added_point);
      void makeYamlObstaclesMarkers(const float min_x, const float max_x, const float min_y, const float max_y, const float cen_x, const float cen_y);
      void makeClickedObstaclesMarkers();

      // Add by r.kobayashi 2024/7/5
      int  pointToCell(float coordinate, float origin, float resolution, int max_index);
      void drawObstacles();
      void publishEnhancedMap();
      void publishObstacleMarkers();

      std::string fn_;
      bool is_adding_;

      std::vector<geometry_msgs::Point> obstacles;
      std::vector<visualization_msgs::Marker> obstacle_markers_; // markers for obstacles
      float obstacle_radius;
      bool debug;
      std::string input_map;
      nav_msgs::OccupancyGrid original_map;
      nav_msgs::MapMetaData map_metadata;
      nav_msgs::OccupancyGrid enhanced_map;
};

} // end namespace ros_augmented_gridmaps


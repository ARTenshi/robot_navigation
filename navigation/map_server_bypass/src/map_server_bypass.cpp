#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"

int occupied_threshold = 70;

ros::Publisher map_pub;

nav_msgs::OccupancyGrid map_msg;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {

  map_msg.header = msg->header;
  map_msg.header.stamp = ros::Time::now();
  map_msg.info = msg->info;
  std::vector<int8_t> arr;
  
  for (int i = 0; i < msg->data.size(); i++) {
    if ( (int)msg->data[i] > occupied_threshold) {
      arr.push_back(100);
    } else if ( (int)msg->data[i] >= 0) {
      arr.push_back( 0 );
    } else {
      //arr.push_back( (int)msg->data[i] );
      arr.push_back( 0 );
    }
  }
  map_msg.data = arr;
  
  map_pub.publish(map_msg);
}

bool srvStaticMapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
  res.map = map_msg;

  return true;
}

bool srvCartographerMapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
  res.map = map_msg;

  return true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "map_server_bypass");
  
  ros::NodeHandle n("~");

  if(ros::param::has("~occupied_threshold"))
      ros::param::get("~occupied_threshold", occupied_threshold);

  ros::Subscriber map_sub = n.subscribe("/map", 10, mapCallback);
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("/cartographer_map", 1);

  ros::ServiceServer service_static_map = n.advertiseService("/static_map", srvStaticMapCallback);
  
  ros::ServiceServer service_cartographer_map = n.advertiseService("/get_cartographer_map", srvCartographerMapCallback);
  
  ros::spin();
  
  return 0;
}


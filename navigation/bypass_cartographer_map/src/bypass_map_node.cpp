#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

ros::Publisher map_pub;

int threshold = 70;

void callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  nav_msgs::OccupancyGrid pub_msg;
  pub_msg.header = msg->header;
  pub_msg.header.stamp = ros::Time::now();
  pub_msg.info = msg->info;
  std::vector<int8_t> arr;
  
  for (int i = 0; i < msg->data.size(); i++) {
    if ( (int)msg->data[i] > threshold) {
      arr.push_back(100);
    } else {
      arr.push_back( (int)msg->data[i] );
    }
  }
  pub_msg.data = arr;
  map_pub.publish(pub_msg);
}


int main(int argc, char **argv){

  ros::init(argc, argv, "map_bypass");
  
  ros::NodeHandle n;

  ros::Subscriber map_sub = n.subscribe("map", 10, callback);
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("update_map", 1);
  
  ros::spin();
  
  return 0;
}

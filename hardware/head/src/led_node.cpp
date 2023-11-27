#include <ros/ros.h>
#include <std_msgs/String.h>
// #include <std_msgs/Float32MultiArray.h>
#include <tmc_msgs/SetColor.h>
#include <iostream>
#include "actionlib_msgs/GoalStatus.h"

#define RATE 30

ros::ServiceClient client;
bool talk_flag = true;

float mapToFloat(short int value)
{
  return static_cast<float>(value) / 255.0;
}

void service_caller(float r, float g, float b)
{
  // Create a request message for the service
  tmc_msgs::SetColor srv;
  srv.request.color.r = r;
  srv.request.color.g = g;
  srv.request.color.b = b;

  // Call the service
  if (client.call(srv))
    std::cout << "Led color changed" << std::endl;
  else
    std::cout << "Failed changing led color" << std::endl;
}

void nav_msg_Callback(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
  
  float r;
  float g;
  float b;
  switch(msg -> status)
  {
    case actionlib_msgs::GoalStatus::ACTIVE:
      r = mapToFloat(255);
      g = r;
      b = 0.0;
      break;
    case actionlib_msgs::GoalStatus::SUCCEEDED:
      r = 0.0;
      g = mapToFloat(255);
      b = 0.0;
      break;
    case actionlib_msgs::GoalStatus::ABORTED:
      r = mapToFloat(139);
      g = 0.0;
      b = 0.0;
      break;
    case actionlib_msgs::GoalStatus::REJECTED:
      r = mapToFloat(85);
      g = r;
      b = r;
      break;
  }
  service_caller(r,g,b);

}

void talk_now_Callback(const std_msgs::String::ConstPtr& msg)
{
  std::string data;
  data = msg -> data;
  if(data == "start"){
    float r = mapToFloat(102);
    float g = mapToFloat(204);
    float b = mapToFloat(255);
    service_caller(r,g,b);
  }
  else{
    float r = 0.0;
    float g = 0.0;
    float b = 1.0;
    service_caller(r,g,b);
  }
  talk_flag = !talk_flag;

}

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "service_client_node");
  ros::NodeHandle n("~");

  client = n.serviceClient<tmc_msgs::SetColor>("/hsrb/status_led_node/set_color");

  ros::Subscriber sub_nav_status  = n.subscribe("/navigation/status", 10, nav_msg_Callback);
  ros::Subscriber sub_talk_now    = n.subscribe("/talk_now", 10, talk_now_Callback);

  ros::Rate loop(RATE);
  while(ros::ok())
  {
    ros::spinOnce();
    loop.sleep();
  }
  

  return 0;
}

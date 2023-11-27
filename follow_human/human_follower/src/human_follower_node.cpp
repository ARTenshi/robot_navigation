#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float32.h"

ros::NodeHandle* n;
ros::Subscriber  sub_legs_pose;
ros::Publisher   pub_cmd_vel;
ros::Publisher   pub_head_pose;
ros::Subscriber sub_pot_fields;
bool move_head = false;
bool pot_fields = false;


float rej_force=0.0;

//pot fields constant
float k_linear_y=1.1;

geometry_msgs::Twist calculate_speeds(float goal_x, float goal_y)
{
    //Control constants
    //float alpha = 0.6548;
    float alpha =0.8;//= 0.9;
    float beta = 0.3;
    float max_angular = 0.6;//0.7


    //Error calculation
    float angle_error = atan2(goal_y, goal_x);
    float distance    = sqrt(goal_x*goal_x + goal_y*goal_y);
    distance -= 0.9;
    if(distance <   0) distance = 0; //Robot will stop at 0.8 m from walker
    if(distance > 0.35) distance = 0.35; //Distance is used as speed, so, robot will move at 0.5 max
    geometry_msgs::Twist result;
    if(distance > 0)
    {
	result.linear.x  = distance * exp(-(angle_error * angle_error) / alpha);
	result.linear.y  = pot_fields? k_linear_y*rej_force:0;
	result.angular.z = max_angular * (2 / (1 + exp(-angle_error / beta)) - 1);
    }
    else
    {
	result.linear.x  = 0;
	result.linear.y  = 0;
	result.angular.z = max_angular * (2 / (1 + exp(-angle_error / beta)) - 1);
    }
    return result;
}

void callback_legs_pose(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if(msg->header.frame_id.compare("base_range_sensor_link") != 0)
    {
	std::cout << "LegFinder.->WARNING!! Leg positions must be expressed wrt robot" << std::endl;
	return;
    }
    pub_cmd_vel.publish(calculate_speeds(msg->point.x, msg->point.y));
    if(move_head)
    {
	std_msgs::Float64MultiArray head_poses;
	head_poses.data.push_back(atan2(msg->point.y, msg->point.x));
	//head_poses.data.push_back(-0.2);
	head_poses.data.push_back(-0.8);
	pub_head_pose.publish(head_poses);
	//std::cout << "head_poses: " << head_poses << std::endl;
    }
}
void callback_pot_fields(const std_msgs::Float32::ConstPtr& msg){

    rej_force=msg->data;
}

void callback_enable(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
	std::cout << "LegFinder.->Enable recevied" << std::endl;
	sub_legs_pose = n->subscribe("/hri/leg_finder/leg_poses", 1, callback_legs_pose);      
	pub_cmd_vel   = n->advertise<geometry_msgs::Twist>("/hsrb/command_velocity", 1);
	pub_head_pose = n->advertise<std_msgs::Float64MultiArray>("/hardware/head/goal_pose", 1);
	sub_pot_fields = n->subscribe("/navigation/obs_avoid/pot_fields/rejective_force", 1, callback_pot_fields);
    }
    else
    {
	sub_legs_pose.shutdown();
	pub_cmd_vel.shutdown();
	pub_head_pose.shutdown();
	sub_pot_fields.shutdown();
    }
}


int main(int argc, char** argv)
{
    move_head = false;
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if(strParam.compare("--move_head") == 0)
            move_head = true;
        if(strParam.compare("--pot_fields")==0){
            pot_fields = true;
            std::cout<<"pot fields activated"<<std::endl;
        }
    }
    
    std::cout << "INITIALIZING HUMAN FOLLOWER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "human_follower");
    n = new ros::NodeHandle();
    ros::Subscriber sub_enable = n->subscribe("/hri/human_following/start_follow", 1, callback_enable);
    ros::Rate loop(20);


    if (n->getParam("/hri/human_following/k_pot_fields", k_linear_y)){
        ROS_INFO("Got param k_linear_y: %f", k_linear_y);
    } 
    else{
        ROS_INFO("Param default k_linear_y: %f", k_linear_y);
    }


    while(ros::ok())
    {        
        ros::spinOnce();
        loop.sleep();
    }
    delete n;
}

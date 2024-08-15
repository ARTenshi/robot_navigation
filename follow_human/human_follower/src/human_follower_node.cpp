#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float32.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"


ros::NodeHandle* n;
ros::Subscriber  sub_legs_pose;
ros::Publisher   pub_cmd_vel;
ros::Publisher   pub_head_pose;
tf::TransformListener* listener;
std::string legs_pose_topic = "/hri/leg_finder/leg_pose";
//Potential fields can be used only with an omnidirectional base.
float repulsiveForce = 0.0;

//Values passed as parameters
float control_alpha  = 0.6548;// 0.6548 ;//= 0.9; // = 1.2
float control_beta   = 0.3;
float max_linear     = 0.3;
float max_angular    = 0.7;//0.7 // 0.8 // 0.7
float dist_to_human  = 0.9;
bool  move_backwards = false;
bool  move_head      = false;
bool pot_fields = false;

bool new_legs_pose = false;
bool enable = false;

geometry_msgs::Twist calculate_speeds(float goal_x, float goal_y)
{
    float angle_error = atan2(goal_y, goal_x);
    float distance    = sqrt(goal_x*goal_x + goal_y*goal_y);
    distance -= dist_to_human;
    if(!move_backwards)
        if(distance <   0) distance = 0;
    if(distance > max_linear) distance = max_linear;
    geometry_msgs::Twist result;
    if(distance > 0 || move_backwards)
    {
        result.linear.x  = distance * exp(-(angle_error * angle_error) / control_alpha);
        result.linear.y  = 0;
        if(pot_fields)
            result.linear.y = repulsiveForce;
        //std::cout << result.linear.y << std::endl;
        result.angular.z = max_angular * (2 / (1 + exp(-angle_error / control_beta)) - 1);
    }
    else
    {
        result.linear.x  = 0;
        result.linear.y  = 0;
        if(fabs(angle_error) >= M_PI_4 / 6.0f)
            result.angular.z = max_angular * (2 / (1 + exp(-angle_error / control_beta)) - 1);
        else
            result.angular.z = 0;
    }
    return result;
}

void transform_to_robot_position(float x, float y, std::string frame_id, float& x_wrt_robot, float& y_wrt_robot)
{
    tf::StampedTransform tf;
    listener->lookupTransform("base_link", frame_id, ros::Time(0), tf);
    Eigen::Affine3d e;
    tf::transformTFToEigen(tf, e);
    Eigen::Vector3d v(x, y, 0);
    v = e * v;
    x_wrt_robot = v.x();
    y_wrt_robot = v.y();
}

void callback_legs_pose(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    float human_x = msg->point.x;
    float human_y = msg->point.y;
    if(msg->header.frame_id.compare("base_link") != 0)
    {
        //std::cout << "LegFinder.->WARNING!! Leg positions must be expressed wrt robot" << std::endl;
        transform_to_robot_position(human_x, human_y, msg->header.frame_id, human_x, human_y);
    }
    if(move_head)
    {
	std_msgs::Float64MultiArray head_poses;
	head_poses.data.push_back(atan2(msg->point.y, msg->point.x));
	head_poses.data.push_back(-0.6);
	pub_head_pose.publish(head_poses);
    }
    pub_cmd_vel.publish(calculate_speeds(human_x, human_y));
    new_legs_pose = true;
}

void callback_rejection_force(const geometry_msgs::Vector3::ConstPtr& msg)
{
    geometry_msgs::Vector3 rejection_force = *msg;
    repulsiveForce = rejection_force.y;
}

void callback_enable(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
        std::cout << "LegFinder.->Enable recevied" << std::endl;
        sub_legs_pose = n->subscribe(legs_pose_topic, 1, callback_legs_pose);      
    }
    else
        sub_legs_pose.shutdown();
    enable = msg->data;
}

void callback_stop(const std_msgs::Empty::ConstPtr& msg)
{
    sub_legs_pose.shutdown();
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING HUMAN FOLLOWER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "human_follower");
    
    n = new ros::NodeHandle();
    std::string cmd_vel_topic   = "/cmd_vel";
    std::string head_topic   = "/hardware/head/goal_pose";
    
    if(ros::param::has("~control_alpha"))
        ros::param::get("~control_alpha", control_alpha);
    if(ros::param::has("~control_beta"))
        ros::param::get("~control_beta", control_beta);
    if(ros::param::has("~max_linear"))
        ros::param::get("~max_linear", max_linear);
    if(ros::param::has("~max_angular"))
        ros::param::get("~max_angular", max_angular);
    if(ros::param::has("~dist_to_human"))
        ros::param::get("~dist_to_human", dist_to_human);
    if(ros::param::has("~move_backwards"))
        ros::param::get("~move_backwards", move_backwards);
    if(ros::param::has("~move_head"))
        ros::param::get("~move_head", move_head);
    if(ros::param::has("~pot_fields"))
        ros::param::get("~pot_fields", pot_fields);
    if(ros::param::has("~legs_pose_topic"))
        ros::param::get("~legs_pose_topic", legs_pose_topic);
    if(ros::param::has("~cmd_vel_topic"))
        ros::param::get("~cmd_vel_topic", cmd_vel_topic);
    if(ros::param::has("~head_topic"))
        ros::param::get("~head_topic", head_topic);
    
    ros::Subscriber sub_rejection_force  = n->subscribe("/navigation/potential_fields/pf_rejection_force", 1, callback_rejection_force);
    ros::Subscriber sub_enable = n->subscribe("/hri/human_following/enable", 1, callback_enable);
    ros::Subscriber sub_stop   = n->subscribe("/stop", 1, callback_stop);
    listener = new tf::TransformListener();
    pub_cmd_vel   = n->advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    pub_head_pose = n->advertise<std_msgs::Float64MultiArray>(head_topic, 1);
    
    ros::Rate loop(30);

    std::cout << "HumanFollower.-> max_linear="<<max_linear<<"  max_angular="<<max_angular<<"  alpha="<<control_alpha<<"  beta="<<control_beta<<std::endl;
    std::cout << "HumanFollower.-> cmd_vel topic: " << cmd_vel_topic << "   legs_pose_topic: " << legs_pose_topic << std::endl;
    int no_legs = 0;
    while(ros::ok())
    {
        if(enable)
        {
            if(new_legs_pose)
                no_legs = 0;
            else
            {
                if(++no_legs > 30)
                {
                    no_legs = 0;
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    pub_cmd_vel.publish(cmd_vel);
                }
            }
        }
        ros::spinOnce();
        loop.sleep();
    }
    delete n;
}

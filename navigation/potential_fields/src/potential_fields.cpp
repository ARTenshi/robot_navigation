#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Path.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

#define RATE 30

bool  debug = false;
bool  enable = false;
bool use_lidar  = true;
bool use_cloud  = false;
float current_speed_linear  = 0;
float current_speed_angular = 0;
int   cloud_downsampling      = 9;
int   cloud_points_threshold  = 100;
int   lidar_downsampling      = 2;
int   lidar_points_threshold  = 15;
std::string base_link_name      = "base_footprint";
std::string point_cloud_topic   = "/point_cloud";
std::string point_cloud_frame   = "/point_cloud_frame";
std::string laser_scan_topic    = "/scan";
std::string laser_scan_frame    = "laser";
int no_data_cloud_counter  = 0;
int no_data_lidar_counter  = 0;


ros::NodeHandle* nh;
ros::Subscriber sub_point_cloud;
ros::Subscriber sub_lidar;
sensor_msgs::PointCloud2::Ptr point_cloud_ptr ;
sensor_msgs::LaserScan::Ptr   laser_scan_ptr  ;
tf::TransformListener* tf_listener;

Eigen::Affine3d get_transform_to_basefootprint(std::string link_name)
{
    tf::StampedTransform tf;
    tf_listener->lookupTransform(base_link_name, link_name, ros::Time(0), tf);
    Eigen::Affine3d e;
    tf::transformTFToEigen(tf, e);
    return e;
}

void get_robot_pose(float& robot_x, float& robot_y, float& robot_t)
{
    tf::StampedTransform transform;
    tf_listener->lookupTransform("map", base_link_name, ros::Time(0), transform);
    robot_x = transform.getOrigin().x();
    robot_y = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    robot_t = atan2((float)q.z(), (float)q.w()) * 2;
}

void callback_lidar(sensor_msgs::LaserScan::Ptr msg)
{
    laser_scan_ptr = msg;
    no_data_lidar_counter = 0;
}

void callback_point_cloud(sensor_msgs::PointCloud2::Ptr msg)
{
    point_cloud_ptr = msg;
    no_data_cloud_counter = 0;
}

void callback_goal_path(const nav_msgs::Path::ConstPtr& msg)
{
     global_goal_x = msg->poses[msg->poses.size() - 1].pose.position.x;
     global_goal_y = msg->poses[msg->poses.size() - 1].pose.position.y;
}

void callbackEnable(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
        std::cout << "ObsDetector.->Starting obstacle detection using: " << (use_lidar ? "lidar " : "") << (use_cloud ? "point_cloud" : "") <<std::endl;
        if(use_cloud )  sub_point_cloud  = nh->subscribe(point_cloud_topic , 1, callback_point_cloud );
        if(use_lidar )  sub_lidar        = nh->subscribe(laser_scan_topic , 1, callback_lidar);
    }
    else
    {
        std::cout << "ObsDetector.->Stopping obstacle detection..." <<std::endl;
        if(use_cloud)   sub_point_cloud .shutdown();
        if(use_lidar)   sub_lidar       .shutdown();
    }
    enable = msg->data;
}

void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
    current_speed_linear  = msg->linear.x;
    current_speed_angular = msg->angular.z;
}

geometry_msgs::Vector3 rejection_force_with_lidar(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    
}

geometry_msgs::Vector3 rejection_force_with_cloud(sensor_msgs::PointCloud2::Ptr msg)
{
}


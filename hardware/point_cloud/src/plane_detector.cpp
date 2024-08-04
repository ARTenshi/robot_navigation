#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <std_msgs/Float32.h>

ros::Publisher pub;
ros::Publisher pub_height;
float threshold = 0.1;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
  try{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, cloud);

    if (cloud.empty()){
      ROS_WARN("Received empty point cloud!");
      return;
    }
    else {
      pcl::ModelCoefficients coefficients;
      pcl::PointIndices inliers;
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.01);

      seg.setInputCloud (cloud.makeShared ());
      seg.segment (inliers, coefficients);

      // Publish the model coefficients
      if (std::abs(coefficients.values[0]) < threshold && std::abs(coefficients.values[1]) < threshold){
        pcl_msgs::ModelCoefficients ros_coefficients;
        pcl_conversions::fromPCL(coefficients, ros_coefficients);
        std_msgs::Float32 height;
        height.data = -coefficients.values[3] / coefficients.values[2];
        pub.publish (ros_coefficients);
        pub_height.publish (height);
      }
    }
  } catch (const std::exception& e) {
    ROS_ERROR("An exception occurred: %s", e.what());
  }

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_plane_detector");
  ros::NodeHandle nh;

  ROS_INFO("ROS NODE: Horizontal plane detector");
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/filtered_point_cloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("/horizontal_plane_coefficients", 1);

  pub_height = nh.advertise<std_msgs::Float32> ("/plane_height", 1);
  // Spin
  ros::spin ();
}

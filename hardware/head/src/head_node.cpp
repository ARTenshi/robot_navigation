/// @brief Copyright (C) 2016 Toyota Motor Corporatio
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include "actionlib_msgs/GoalStatus.h"

float goalTilt;
float goalPan;

bool isNewData;

std_msgs::Float64MultiArray msg_hd_cp;

void headGoalPoseCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  isNewData = true;
        goalPan = msg->data[0];
        goalTilt = msg->data[1];

        if(goalPan > 1.74 )
                goalPan = 1.74;

        if(goalPan < -3.141592 )
                goalPan = -3.141592;

        if(goalTilt > 0.47)
                goalTilt = 0.47;

        if(goalTilt < -0.9)
                goalTilt = -0.9;
}

void headCurrentPoseCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
        // std::cout << "current pose: [  pan,         tilt  ]" << std::endl
        //      << "            " << msg->actual.positions[1]
        //      << " " << msg->actual.positions[0] << std::endl;

        msg_hd_cp.data.resize(2);
        msg_hd_cp.data[0] = msg->actual.positions[1]; // Current pan pose
        msg_hd_cp.data[1] = msg->actual.positions[0]; // current tilt pose
}

void nav_msg_Callback(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
  switch(msg -> status)
  {
    case actionlib_msgs::GoalStatus::ACTIVE:
      break;
    default:
        isNewData = true;
        goalPan = 0.0;
        goalTilt = 0.0;
  }
}


int main(int argc, char **argv)
{
        std::cout << std::endl << "--------------------->" << std::endl;
        std::cout << "INITIALIZING HEAD_BRIDGE_NODE BY EDD-II" << std::endl;
        ros::init(argc, argv, "head_bridge_node");

        trajectory_msgs::JointTrajectory traj;
        controller_manager_msgs::ListControllers list_controllers;


        // initalize ROS publisher
        ros::NodeHandle n;
        ros::Publisher pub_pumas_head_cp;
        ros::Publisher pub_hsr_head_gp;
        ros::Subscriber sub_pumas_head_gp;
        ros::Subscriber sub_hsr_head_cp;
        ros::Subscriber sub_nav_finished;

        ros::ServiceClient client;
        ros::Rate loop(30);

        pub_hsr_head_gp = n.advertise<trajectory_msgs::JointTrajectory>("/hsrb/head_trajectory_controller/command", 10);
        pub_pumas_head_cp = n.advertise<std_msgs::Float64MultiArray>("/hardware/head/current_pose", 10);


        sub_pumas_head_gp = n.subscribe("/hardware/head/goal_pose", 10, headGoalPoseCallback);
        sub_hsr_head_cp = n.subscribe("/hsrb/head_trajectory_controller/state", 10, headCurrentPoseCallback);
        sub_nav_finished = n.subscribe("/navigation/status", 10, nav_msg_Callback);

        // make sure the controller is running
        client = n.serviceClient<controller_manager_msgs::ListControllers>("/hsrb/controller_manager/list_controllers");


        // wait to establish connection between the controller
        while (pub_hsr_head_gp.getNumSubscribers() == 0) {
                ros::Duration(0.1).sleep();
        }


        bool running = false;
        while (running == false) {
                ros::Duration(0.1).sleep();
                if (client.call(list_controllers)) {
                        for (unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
                                controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
                                if (c.name == "head_trajectory_controller" && c.state == "running") {
                                        running = true;
                                }
                        }
                }
        }


        traj.joint_names.push_back("head_pan_joint");
        traj.joint_names.push_back("head_tilt_joint");

        // Resize message
        traj.points.resize(1);
        traj.points[0].positions.resize(2);
        traj.points[0].velocities.resize(2);

        // Set velocities
        for (size_t i = 0; i < 2; ++i) {
                traj.points[0].velocities[i] = 0.1;
        }

        traj.points[0].time_from_start = ros::Duration(0.05);

        while(ros::ok())
        {
                traj.points[0].positions[0] = goalPan;
                traj.points[0].positions[1] = goalTilt;

                // publish ROS message
                if(pub_hsr_head_gp.getNumSubscribers() > 0 && isNewData)
		{
		  pub_hsr_head_gp.publish(traj);
		  isNewData = false;
		}

                if(pub_pumas_head_cp.getNumSubscribers() > 0)
                        pub_pumas_head_cp.publish(msg_hd_cp);

		
                loop.sleep();
                ros::spinOnce();
        }


        return 0;
}

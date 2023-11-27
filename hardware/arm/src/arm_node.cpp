/// @brief Copyright (C) 2016 Toyota Motor Corporation
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <tmc_control_msgs/GripperApplyEffortGoal.h>
#include <tmc_control_msgs/GripperApplyEffortAction.h>
//#include <tmc_suction/SuctionControlAction.h>
//#include <tmc_suction/SuctionControlGoal.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>


float torso_goal_pose;

std::vector<float>  arm_goal_pose;
std::vector<float>  arm_complete_cp;
std::vector<float>  gripper_goal_pose;
bool msg_arm_recived        = true;
bool msg_torso_recived      = false;
bool msg_gripper_recived    = true;
bool msg_gripp_torq_recived = false;
bool msg_suction_recived    = false;
bool wait_for_arm_goal_pose = false;
bool wait_for_torso_goal_pose = false;
bool nav_request_received     = true;

tmc_control_msgs::GripperApplyEffortGoal goal;
//tmc_suction::SuctionControlGoal goal_suction;


/*void armSuctionCallback(const std_msgs::Bool::ConstPtr& msg)
{
  goal_suction.suction_on.data = msg->data;

  if (msg->data)
    std::cout << "Takeshi Hardware->  SuctionGripper is  [ACTIVE]" << std::endl;
  else
    std::cout << "Takeshi Hardware->  SuctionGripper is  [INACTIVE]" << std::endl;

  msg_suction_recived = true;
}*/



void armGoalPoseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  if(msg->data.size() != 4)
  {
    std::cout << "[ARM-NODE]: Error data size." << std::endl;
    std::cout << "[ARM-NODE]: Arm data must be four values." << std::endl;
  }
  else
    msg_arm_recived = true;

  arm_goal_pose.resize(4);

  if(msg_arm_recived)
  {
    for(int i = 0; i < 4; i++)
      arm_goal_pose[i] = msg->data[i];
  }
}

void gripperPoseCallback(const std_msgs::Float32::ConstPtr& msg)
{
  // Expected value between [0.0 - 1.0] where 0.0 is close gripper and 1.0 is totally open gripper
  gripper_goal_pose.resize(1);
  gripper_goal_pose[0] = -0.105 + (msg->data)*( 1.23 + 0.105);  // Joint limits from official page

  msg_gripper_recived = true;
}

void gripperTorqueCallback(const std_msgs::Float32::ConstPtr& msg)
{
  goal.effort = msg->data * (-1);
  msg_gripp_torq_recived = true;
}


void torsoGoalPoseCallback(const std_msgs::Float32::ConstPtr& msg)
{
  torso_goal_pose = msg->data;
  msg_torso_recived = true;
}

void armCurrentPoseCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
  // Read five values of torso and arm
  arm_complete_cp.resize(5);
  arm_complete_cp[0] = msg->actual.positions[0];   // torso joint
  arm_complete_cp[1] = msg->actual.positions[1];
  arm_complete_cp[2] = msg->actual.positions[2];
  arm_complete_cp[3] = msg->actual.positions[3];
  arm_complete_cp[4] = msg->actual.positions[4];
}

void nav_msg_Callback(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
  switch(msg -> status)
  {
    case actionlib_msgs::GoalStatus::ACTIVE:
      nav_request_received = true;
      break;
  }
}

int main(int argc, char **argv)
{
  std::cout << std::endl << "------------------------->" << std::endl;
  std::cout << "INITIALIZING ARM_BRIDGE_NODE BY [EDD-II] " << std::endl;
  ros::init(argc, argv, "arm_bridge");

  std_msgs::Float32 msg_torso_current_pose;
  std_msgs::Float32MultiArray msg_arm_current_pose;
  std_msgs::Bool msg_arm_goal_pose;
  std_msgs::Bool msg_torso_goal_pose;

  // initalize ROS publisher
  ros::NodeHandle  n;
  ros::Publisher   pub_hsr_arm_gp;
  ros::Publisher   pub_hsr_gripp_gp;
  ros::Publisher   pub_torso_curren_pose;
  ros::Publisher   pub_arm_curren_pose;
  ros::Publisher   pub_arm_goal_pose;
  ros::Publisher   pub_torso_goal_pose;

  ros::Subscriber  sub_pumas_arm_gp;
  ros::Subscriber  sub_pumas_gripp_gp;
  ros::Subscriber  sub_pumas_torque_gripp;
  //ros::Subscriber  sub_pumas_suction_gripp;
  ros::Subscriber  sub_torso_goal_pose;
  ros::Subscriber  sub_hsr_arm_cp;
  ros::Subscriber  sub_start_nav;



  actionlib::SimpleActionClient<tmc_control_msgs::GripperApplyEffortAction>
    gripperActionClient("/hsrb/gripper_controller/grasp", true);

 /* actionlib::SimpleActionClient<tmc_suction::SuctionControlAction>
    suctionActionClient("/hsrb/suction_control");*/

  ros::Rate        loop(60);

  // Publishers for hsr-hardware
  pub_hsr_arm_gp      = n.advertise<trajectory_msgs::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);
  pub_hsr_gripp_gp    = n.advertise<trajectory_msgs::JointTrajectory>("/hsrb/gripper_controller/command", 10);
  pub_torso_curren_pose = n.advertise<std_msgs::Float32>("/hardware/torso/current_pose", 10);
  pub_arm_curren_pose = n.advertise<std_msgs::Float32MultiArray>("/hardware/arm/current_pose", 10);
  pub_arm_goal_pose   = n.advertise<std_msgs::Bool>("/hardware/arm/armGoalPose", 10);
  pub_torso_goal_pose = n.advertise<std_msgs::Bool>("/hardware/torso/goal_reached", 10);


  sub_pumas_arm_gp        = n.subscribe("/hardware/arm/goal_pose", 10, armGoalPoseCallback);
  sub_pumas_gripp_gp      = n.subscribe("/hardware/arm/goal_gripper", 10, gripperPoseCallback);
  sub_pumas_torque_gripp  = n.subscribe("/hardware/arm/torque_gripper", 10, gripperTorqueCallback);
  //sub_pumas_suction_gripp = n.subscribe("/hardware/arm/gripper_suction", 10, armSuctionCallback);
  sub_torso_goal_pose     = n.subscribe("/hardware/torso/goal_pose", 10, torsoGoalPoseCallback);
  sub_hsr_arm_cp          = n.subscribe("/hsrb/arm_trajectory_controller/state", 10, armCurrentPoseCallback);
  sub_start_nav           = n.subscribe("/navigation/status", 10, nav_msg_Callback);

    // wait for the action server to start
  gripperActionClient.waitForServer();
  //suctionActionClient.waitForServer();


  // wait to establish connection between the controller
  while (pub_hsr_arm_gp.getNumSubscribers() == 0) {
    ros::Duration(0.1).sleep();
  }


  // make sure the controller is running
  ros::ServiceClient client = n.serviceClient<controller_manager_msgs::ListControllers>(
      "/hsrb/controller_manager/list_controllers");
  controller_manager_msgs::ListControllers list_controllers;
  bool running = false;
  while (running == false) {
    ros::Duration(0.1).sleep();
    if (client.call(list_controllers)) {
      for (unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
        controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
        if (c.name == "arm_trajectory_controller" && c.state == "running") {
          running = true;
        }
      }
    }
  }

  // fill ROS message
  trajectory_msgs::JointTrajectory traj_arm;
  trajectory_msgs::JointTrajectory traj_gripp;

  traj_arm.joint_names.push_back("arm_lift_joint");
  traj_arm.joint_names.push_back("arm_flex_joint");
  traj_arm.joint_names.push_back("arm_roll_joint");
  traj_arm.joint_names.push_back("wrist_flex_joint");
  traj_arm.joint_names.push_back("wrist_roll_joint");
  traj_gripp.joint_names.push_back("hand_motor_joint");    // Gripper trayectorie msgs

  traj_arm.points.resize(1);
  traj_gripp.points.resize(1);

  traj_arm.points[0].positions.resize(5);
  traj_arm.points[0].velocities.resize(5);
  traj_gripp.points[0].positions.resize(1);
  traj_gripp.points[0].velocities.resize(1);
  traj_gripp.points[0].effort.resize(1);

  //msg_torso_current_pose.data.resize(1);
  msg_arm_current_pose.data.resize(4);


  // Initial values
  traj_arm.points[0].velocities[0] = 0.0;
  traj_arm.points[0].velocities[1] = 0.0;
  traj_arm.points[0].velocities[2] = 0.1;
  traj_arm.points[0].velocities[3] = 0.1;
  traj_arm.points[0].velocities[4] = 0.1;
  traj_gripp.points[0].velocities[0] = -0.5;

  traj_arm.points[0].positions[0] = 0.0;
  traj_arm.points[0].positions[1] = 0.0;
  traj_arm.points[0].positions[2] = -1.5707;
  traj_arm.points[0].positions[3] = -1.5707;
  traj_arm.points[0].positions[4] = 0.0;
  traj_gripp.points[0].positions[0] = -0.4;

  traj_gripp.points[0].effort[0] = -0.2;

  traj_arm.points[0].time_from_start = ros::Duration(3.0);
  traj_gripp.points[0].time_from_start = ros::Duration(3.0);

  arm_goal_pose.resize(5);
  arm_complete_cp.resize(5);
  gripper_goal_pose.resize(1);


  arm_goal_pose[0] = 0.0;
  arm_goal_pose[1] = -1.5707;
  arm_goal_pose[2] = -1.5707;
  arm_goal_pose[3] = 0.0;


  while(ros::ok())
  {
    // Set goal_pose
    for (size_t i = 0; i < 4; ++i)
    traj_arm.points[0].positions[i+1] = arm_goal_pose[i];

    traj_gripp.points[0].positions[0] = gripper_goal_pose[0];
    traj_arm.points[0].positions[0] = torso_goal_pose;

    // Set current pose for publish
    msg_torso_current_pose.data = arm_complete_cp[0];
    msg_arm_current_pose.data[0] = arm_complete_cp[1];
    msg_arm_current_pose.data[1] = arm_complete_cp[2];
    msg_arm_current_pose.data[2] = arm_complete_cp[3];
    msg_arm_current_pose.data[3] = arm_complete_cp[4];


    // publish ROS message
    if(msg_arm_recived)
    {
      pub_hsr_arm_gp.publish(traj_arm);
      msg_arm_recived = false;
      wait_for_arm_goal_pose = true;
    }

    if(msg_torso_recived)
    {
      pub_hsr_arm_gp.publish(traj_arm);
      msg_torso_recived = false;
      wait_for_torso_goal_pose = true;
    }

    if(msg_gripper_recived)
    {
      pub_hsr_gripp_gp.publish(traj_gripp);
      msg_gripper_recived = false;
    }

    if(msg_gripp_torq_recived)
    {
      gripperActionClient.sendGoalAndWait(goal);
      msg_gripp_torq_recived = false;
    }

    if(nav_request_received)
    {
      pub_hsr_arm_gp.publish(traj_arm);
      nav_request_received = false;
    }

    /*if(msg_suction_recived)
    {
      suctionActionClient.sendGoalAndWait(goal_suction);
      msg_suction_recived = false;
    }*/

    if(pub_torso_curren_pose.getNumSubscribers() > 0)
      pub_torso_curren_pose.publish(msg_torso_current_pose);

    if(pub_arm_curren_pose.getNumSubscribers() > 0)
      pub_arm_curren_pose.publish(msg_arm_current_pose);


    if(wait_for_arm_goal_pose){
      float arm_d=0;
      for(size_t i=0; i < msg_arm_current_pose.data.size(); i++)
        arm_d=arm_d+pow(traj_arm.points[0].positions[i+1] - msg_arm_current_pose.data[i],2);
      arm_d=pow(arm_d,0.5);
      if(arm_d < 0.01){
        std::cout << "Wait for arm goal pose: true" << std::endl;
        msg_arm_goal_pose.data=true;
        pub_arm_goal_pose.publish(msg_arm_goal_pose);
        wait_for_arm_goal_pose = false;
      }
    }

    if(wait_for_torso_goal_pose){
      float torso_d=0;
      torso_d=fabs(traj_arm.points[0].positions[0] - msg_torso_current_pose.data);
      if(torso_d < 0.01){
        std::cout << "Wait for torso goal pose: true " << torso_d << std::endl;
        msg_torso_goal_pose.data=true;
        pub_torso_goal_pose.publish(msg_torso_goal_pose);
        wait_for_torso_goal_pose = false;
      }
    }

    loop.sleep();
    ros::spinOnce();
  }
  return 0;
}

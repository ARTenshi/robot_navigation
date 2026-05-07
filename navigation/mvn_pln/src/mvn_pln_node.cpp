#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetPlan.h"
#include "actionlib_msgs/GoalStatus.h"
#include "tf/transform_listener.h"

#include "actionlib/client/simple_action_client.h"
#include "motion_synth/Joints.h"
#include "motion_synth/StartAndEndJoints.h"
#include "motion_synth/MotionSynthesisAction.h"

#include "path_planner/GetPlanWithVia.h"
#include "geometry_msgs/Pose.h"


#define RATE 10

#define SM_INIT 0
#define SM_WAITING_FOR_TASK 1
#define SM_CALCULATE_PATH 2
#define SM_CHECK_IF_INSIDE_OBSTACLES 19
#define SM_WAITING_FOR_MOVE_BACKWARDS 18
#define SM_CHECK_IF_OBSTACLES 21
#define SM_WAIT_FOR_NO_OBSTACLES 22
#define SM_ENABLE_OBS_DETECT 23
#define SM_WAIT_FOR_OBS_DETECT 24
#define SM_RECOVERY_ROTATE 25
#define SM_START_MOVE_PATH 3
#define SM_WAIT_FOR_MOVE_FINISHED 4
#define SM_COLLISION_DETECTED 5
#define SM_STOP_RECEIVED 51
#define SM_CORRECT_FINAL_ANGLE 6
#define SM_WAIT_FOR_ANGLE_CORRECTED 7
#define SM_FINAL    17

bool stop = false;
bool collision_risk = false;
bool  patience = true;
bool memory_all_obstacles = false;
actionlib_msgs::GoalStatus simple_move_goal_status;
int simple_move_status_id = 0;

bool nav_goal_received = false;
geometry_msgs::Pose global_goal;
std::string base_link_name = "base_footprint";

//update r.kobayashi 2025/6/27
//forcing_backward
bool forcing_backward = false;
float backward_distance = -0.25;
float disable_backward_threshold = 0.7; //nearness to goal, backward behavior is disable, goal error < disable_backward_threshold


// motion_synth
bool arm_goal_received = false;
bool arm_goal_reached = false;
bool has_arm_start_pose = false;
bool has_arm_end_pose = false;
motion_synth::StartAndEndJoints target_arm_pose;

//add by r.k 2025/06/12 via point
std::vector<geometry_msgs::Pose> via_points;
void callback_viapoints(const geometry_msgs::PoseArray::ConstPtr& msg) {
    via_points = msg->poses;
    ROS_INFO("MvnPln.->Received %lu viapoints -> A* with via_points!!!", via_points.size());
}

void callback_general_stop(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "MvnPln.->General Stop signal received" << std::endl;
    stop = true;
}

void callback_navigation_stop(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "MvnPln.->Navigation Stop signal received" << std::endl;
    stop = true;
}

void callback_simple_goal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    global_goal = msg->pose;
    nav_goal_received = true;
}

void callback_arm_joints(const motion_synth::StartAndEndJoints::ConstPtr& msg)
{
    target_arm_pose = *msg;
    arm_goal_received = true;
}

void callback_collision_risk(const std_msgs::Bool::ConstPtr& msg){
    collision_risk = msg->data;
}

void callback_simple_move_goal_status(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
    simple_move_goal_status = *msg;
    std::stringstream ss;
    ss << msg->goal_id.id;
    ss >> simple_move_status_id;
}

void callback_set_patience(const std_msgs::Bool::ConstPtr& msg)
{
    std::cout << "MvnPln.->Set patience: " << (msg->data ? "True" : "False") << std::endl;
    patience = msg->data;
}

// original path_plan implementation
//bool plan_path_from_augmented_map(float robot_x, float robot_y, float goal_x, float goal_y, ros::ServiceClient& clt, nav_msgs::Path& path)
//{
//    nav_msgs::GetPlan srv;
//    srv.request.start.pose.position.x = robot_x;
//    srv.request.start.pose.position.y = robot_y;
//    srv.request.goal.pose.position.x = goal_x;
//    srv.request.goal.pose.position.y = goal_y;
//    if(!clt.call(srv))
//        return false;
//    path = srv.response.plan;
//    return true;
//}

//add by r.k 2025/06/12 for via
bool plan_path_from_augmented_map(float robot_x, float robot_y, float goal_x, float goal_y, ros::ServiceClient& clt, nav_msgs::Path& path)
{
    path_planner::GetPlanWithVia srv;
    geometry_msgs::PoseStamped start_pose, goal_pose;
    start_pose.header.frame_id = "map";
    start_pose.header.stamp = ros::Time::now();
    start_pose.pose.position.x = robot_x;
    start_pose.pose.position.y = robot_y;

    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.pose.position.x = goal_x;
    goal_pose.pose.position.y = goal_y;


    srv.request.start = start_pose;
    srv.request.goal = goal_pose;

    for (const auto& vp : via_points)
    {
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = "map";
        ps.header.stamp = ros::Time::now();
        ps.pose = vp;
        srv.request.via_points.push_back(ps);
    }

    if (!clt.call(srv))
        return false;

    path = srv.response.plan;
    return true;
}

//add by r.k 2025/06/12 for via
void updateViaPoints(const geometry_msgs::Pose& robot, double pass_via_threshold)
{
    if (via_points.empty()) {
        return; // No via points to update
    }

    // check collision_risk, if true, purge all via points
    if (collision_risk) {
        ROS_WARN("MvnPln.->Collision risk detected, purging all via points.");
        via_points.clear();
        return;
    }

    //previous dist
    static double previous_dist = std::numeric_limits<double>::infinity();

    //get first via position
    const auto& v = via_points.front(); // Access the first via point
    double dx = v.position.x - robot.position.x;
    double dy = v.position.y - robot.position.y;
    double robot2via_distance = std::hypot(dx, dy);

    if (robot2via_distance < pass_via_threshold) {
        ROS_INFO ("MvnPln.-> Passed via-point at (%.3f, %.3f) -> Erase via-point", v.position.x, v.position.y);
        via_points.erase(via_points.begin());
        previous_dist = std::numeric_limits<double>::infinity(); // Reset previous distance for next via point
    }

    if (robot2via_distance < previous_dist - 1e-4) {
        previous_dist = robot2via_distance; 
        return;
    }
}

void get_robot_position(tf::TransformListener& listener, float& robot_x, float& robot_y, float& robot_a){
    tf::StampedTransform tf;
    ros::Time now = ros::Time::now();

    if (!listener.waitForTransform("map", base_link_name, now, ros::Duration(0.5))) {
        ROS_WARN("MvnPln.->Timeout waiting for transform from map to %s (within 0.5s)", base_link_name.c_str());
        return;
    }

    try {
        listener.lookupTransform("map", base_link_name, now, tf);

        robot_x = tf.getOrigin().x();
        robot_y = tf.getOrigin().y();

        tf::Quaternion q = tf.getRotation();
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        robot_a = yaw;
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("MvnPln.->TF Exception while transforming from map to %s: %s", base_link_name.c_str(), ex.what());
        return;
    }
}


//void get_robot_position(tf::TransformListener& listener, float& robot_x, float& robot_y, float& robot_a){
//    tf::StampedTransform tf;
//    ros::Time now = ros::Time::now();
//
//    if (listener.canTransform("map", base_link_name, now)){
//
//        try{
//            listener.lookupTransform("map", base_link_name, now, tf);
//        }
//        catch (tf::TransformException &ex) {
//            ROS_WARN("MvnPln.->TF Exception while transforming from map to %s: %s", base_link_name.c_str(), ex.what());
//            return;
//        }
//
//        robot_x = tf.getOrigin().x();
//        robot_y = tf.getOrigin().y();
//
//        tf::Quaternion q = tf.getRotation();
//        double roll, pitch, yaw;
//        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
//        robot_a = yaw;
//    }
//    else {
//        ROS_WARN("MvnPln.->Cannot transform from map to %s, get_robot_position", base_link_name.c_str());
//        return;
//    }
//}

int publish_status(int status, int id, std::string text, ros::Publisher& pub)
{
    actionlib_msgs::GoalStatus msg;
    std::stringstream ss;
    ss << id;
    msg.status = status;
    msg.text   = text;
    ss >> msg.goal_id.id;
    msg.goal_id.stamp = ros::Time::now();
    pub.publish(msg);
    return status;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mvn_pln");
    ROS_INFO("INITIALIZING MOVING PLANNER NODE BY MARCO NEGRETE..." );

    ros::NodeHandle n;
    tf::TransformListener listener;
    
    float proximity_criterion = 0.7;
    //float move_error_threshold = 0.03;

    if(ros::param::has("~patience"))
        ros::param::get("~patience", patience);
    if(ros::param::has("~proximity_criterion"))
        ros::param::get("~proximity_criterion", proximity_criterion);
    if(ros::param::has("/base_link_name"))
        ros::param::get("/base_link_name", base_link_name);
    if(ros::param::has("~forcing_backward"))
        ros::param::get("~forcing_backward", forcing_backward);
    if(ros::param::has("~backward_distance"))
        ros::param::get("~backward_distance", backward_distance);
    if(ros::param::has("~disable_backward_threshold"))
        ros::param::get("~disable_backward_threshold", disable_backward_threshold);
    if(ros::param::has("~memory_all_obstacles"))
        ros::param::get("~memory_all_obstacles", memory_all_obstacles);
    //if(ros::param::has("~move_error_threshold"))
    //    ros::param::get("~move_error_threshold", move_error_threshold);

    std::cout << "MvnPln.->Patience: " << (patience?"True":"False") << "  Proximity criterion: " << proximity_criterion;
    std::cout << "  base link name: " << base_link_name << std::endl;
    //std::cout << "  move error threshold: " << move_error_threshold << std::endl;
    std::cout << "MvnPln.->Waiting for localization transform..." << std::endl;
    listener.waitForTransform("map", base_link_name, ros::Time(0), ros::Duration(10.0));
    std::cout << "MvnPln.->Localization transform is ready ..." << std::endl;
    std::cout << "MvnPln.->Waiting for path planner to be ready..." << std::endl;
    ros::service::waitForService("/path_planner/plan_path_with_static"   , ros::Duration(10));
    ros::service::waitForService("/path_planner/plan_path_with_augmented", ros::Duration(10));
    std::cout << "MvnPln.->Path planner is ready." << std::endl;
    std::cout << "mvnPln.->Waiting for map augmenter to be ready..." << std::endl;
    ros::service::waitForService("/map_augmenter/get_augmented_map"     , ros::Duration(10));
    ros::service::waitForService("/map_augmenter/get_augmented_cost_map", ros::Duration(10));
    ros::service::waitForService("/map_augmenter/are_there_obstacles"   , ros::Duration(10));
    ros::service::waitForService("/map_augmenter/clear_memory_all_obstacles"   , ros::Duration(10));
    std::cout << "MvnPln.->Map Augmenter is ready..." << std::endl;
    
    ros::Subscriber sub_generalStop        = n.subscribe("/stop", 10, callback_general_stop);
    ros::Subscriber sub_navCtrlStop        = n.subscribe("/navigation/stop", 10, callback_navigation_stop);               
    ros::Subscriber sub_simple_goal        = n.subscribe("/nav_control/goal", 10, callback_simple_goal);
    ros::Subscriber sub_arm_goal     = n.subscribe<motion_synth::StartAndEndJoints>("/pumas_motion_synth/joint_pose", 10, callback_arm_joints);
    ros::Subscriber sub_collision_risk     = n.subscribe("/navigation/obs_detector/collision_risk", 10, callback_collision_risk);
    ros::Subscriber sub_move_goal_status   = n.subscribe("/simple_move/goal_reached", 10, callback_simple_move_goal_status);
    ros::Subscriber sub_set_patience       = n.subscribe("/navigation/set_patience", 10, callback_set_patience);
    ros::Subscriber sub_via_points       = n.subscribe("/navigation/via_points", 10, callback_viapoints);
    ros::Publisher pub_obs_detector_enable = n.advertise<std_msgs::Bool>("/navigation/obs_detector/enable", 1);
    ros::Publisher pub_goal_dist_angle     = n.advertise<std_msgs::Float32MultiArray>("/simple_move/goal_dist_angle", 1);
    ros::Publisher pub_status              = n.advertise<actionlib_msgs::GoalStatus>("/navigation/status", 10);
    ros::Publisher pub_simple_move_stop    = n.advertise<std_msgs::Empty>("/simple_move/stop", 1);
    ros::Publisher pub_goal_path           = n.advertise<nav_msgs::Path>("/simple_move/goal_path", 1); //original
    ros::Publisher pub_tmp_head_pose_cancel = n.advertise<std_msgs::Empty>("/navigation/tmp_head_pose_cancel", 1);

    //ros::ServiceClient clt_plan_path       = n.serviceClient<nav_msgs::GetPlan>("/path_planner/plan_path_with_augmented"); //original implementation
    ros::ServiceClient clt_plan_path = n.serviceClient<path_planner::GetPlanWithVia>("/path_planner/plan_path");
    ros::ServiceClient clt_are_there_obs   = n.serviceClient<std_srvs::Trigger>("/map_augmenter/are_there_obstacles");
    ros::ServiceClient clt_is_in_obstacles = n.serviceClient<std_srvs::Trigger>("/map_augmenter/is_inside_obstacles");
    ros::ServiceClient clt_clear_memory_all_obstacles = n.serviceClient<std_srvs::Trigger>("/map_augmenter/clear_memory_all_obstacles");


    // for motion synth_action_client add by r.k 2025/04/10
    typedef actionlib::SimpleActionClient<motion_synth::MotionSynthesisAction> MotionSynthActionClient;
    MotionSynthActionClient* ms_ac;
    ms_ac = new MotionSynthActionClient("/motion_synth", true);
    ROS_INFO("MvnPln.->Waiting for motion_synth action server");
    ms_ac->waitForServer(ros::Duration(5.0));
    ROS_INFO("MvnPln.->Connected to motion_synth action server");
    //

    ros::Rate loop(RATE);
    ros::Rate slow_loop(1);

    float robot_x = 0;
    float robot_y = 0;
    float robot_a = 0;
    float error = 0;

    int  state = SM_INIT;
    int  simple_move_sequencer = -1;
    int  goal_id = -1;
    int  current_status = 0;
    bool near_goal_sent = false;

    std_msgs::Bool msg_bool;
    std_srvs::Trigger srv_check_obstacles;
    nav_msgs::Path path;
    std_msgs::Float32MultiArray msg_goal_dist_angle;
    msg_goal_dist_angle.data.resize(2);

    while(ros::ok())
    {
        //rosparam update dynamically
        ros::param::get("~forcing_backward", forcing_backward);
        
        if(stop)
        {
            stop = false;
            state = SM_INIT;
            if(current_status == actionlib_msgs::GoalStatus::ACTIVE)
                current_status=publish_status(actionlib_msgs::GoalStatus::ABORTED,goal_id, "Stop signal received. Task cancelled", pub_status);
        }

        if(nav_goal_received)
            state = SM_WAITING_FOR_TASK;

        switch(state)
        {
            case SM_INIT:
                ROS_WARN("Pumas Navigation. -> Ready!!!!!!!!! ");
                ROS_WARN("MvnPln.-> MVN PLN READY. Waiting For New Goal.");

                //add by r.k 2025/06/12
                via_points.clear();

                state = SM_WAITING_FOR_TASK;
                break;

            case SM_WAITING_FOR_TASK:
                
                //motion synth add by r.k 2025/04/10
                if (arm_goal_received)
                {
                    motion_synth::MotionSynthesisGoal motion_synth_goal;
                    motion_synth_goal.goal_location.x = global_goal.position.x;
                    motion_synth_goal.goal_location.y = global_goal.position.y;
                    //motion_synth_goal.goal_location.theta = atan2(global_goal.orientation.z, global_goal.orientation.w) * 2;
                    motion_synth_goal.goal_location.theta = tf::getYaw(global_goal.orientation);

                    motion_synth_goal.apply_start_pose = false;
                    motion_synth_goal.apply_goal_pose = false;

                    if (target_arm_pose.has_arm_start_pose)
                    {
                        motion_synth_goal.apply_start_pose = true;
                        motion_synth_goal.start_pose = target_arm_pose.start_pose;
                        ROS_INFO("MvnPln.->Arm motion start_pose sent");
                    }

                    if (target_arm_pose.has_arm_end_pose)
                    {
                        motion_synth_goal.apply_goal_pose = true;
                        motion_synth_goal.goal_pose = target_arm_pose.end_pose;
                        ROS_INFO("MvnPln.->Arm motion end_pose sent");
                    }

                    ms_ac->sendGoal(motion_synth_goal);
                    arm_goal_received = false;
                    motion_synth_goal.apply_start_pose = false;
                    motion_synth_goal.apply_goal_pose = false;
                }

                if(nav_goal_received)
                {
                    nav_goal_received = false;
                    state = SM_CALCULATE_PATH;
                    if(current_status == actionlib_msgs::GoalStatus::ACTIVE)
                        current_status = publish_status(actionlib_msgs::GoalStatus::ABORTED, goal_id, "Cancelling current movement.", pub_status);
                    goal_id++;
                    ROS_INFO("MvnPln.->New goal received. Current task goal_id: ");
                    near_goal_sent = false;
                }
                break;
                
            case SM_CALCULATE_PATH:
            get_robot_position(listener, robot_x, robot_y, robot_a);
            if(!plan_path_from_augmented_map(robot_x, robot_y, global_goal.position.x, global_goal.position.y, clt_plan_path, path))
            {
                std::cout << "MvnPln.->Cannot calc path to " << global_goal.position.x << " " << global_goal.position.y << std::endl;
                pub_simple_move_stop.publish(std_msgs::Empty());

                if (memory_all_obstacles){
                    std_srvs::Trigger clear_memory_all_obstacles_srv;
                    if (clt_clear_memory_all_obstacles.call(clear_memory_all_obstacles_srv)){
                        if (clear_memory_all_obstacles_srv.response.success){
                            ROS_WARN("MvnPln.->Cleared memory obstacles due to path planning failure");
                        }
                        else{
                            ROS_WARN("MvnPln.->Cleared memory obstacles service response failed");
                        }
                    }
                    else{
                        ROS_ERROR("MvnPln.->Failed to call clear_memory_all_obstacles_srv");

                    }
                }

                bool robot_is_in_static_obstacle = false;
                if (clt_is_in_obstacles.call(srv_check_obstacles) && srv_check_obstacles.response.success)
                {
                    robot_is_in_static_obstacle = true;
                }

                if (robot_is_in_static_obstacle)
                {
                    ROS_ERROR("MvnPln.->Robot is inside a static obstacle. Attempting recovery by moving backwards.");
                    state = SM_CHECK_IF_INSIDE_OBSTACLES; //replan
                }
                else
                {
                    if (patience) // If patient, check for temporal obstacles first
                    {
                        ROS_WARN("MvnPln.->Path planning failed, but robot is not in a static obstacle. Checking for temporal obstacles.");
                        state = SM_CHECK_IF_OBSTACLES;
                    }
                    else
                    {
                        ROS_ERROR("MvnPln.->Path planning failed and patience is false. Goal unreachable or in obstacle. Aborting.");
                        current_status = publish_status(actionlib_msgs::GoalStatus::ABORTED, goal_id, "Goal unreachable or in obstacle", pub_status);
                        state = SM_INIT;
                    }
                }
            }
            else
            {
                state = SM_ENABLE_OBS_DETECT;
            }
            break;


            case SM_CHECK_IF_INSIDE_OBSTACLES:
            {
                ROS_INFO("MvnPln.->Checking if robot is inside an obstacle...");
                if (clt_is_in_obstacles.call(srv_check_obstacles)){
                    if(srv_check_obstacles.response.success)
                    {

                        ROS_INFO("MvnPln.->Robot is inside an obstacle. Moving backwards...");
                        msg_goal_dist_angle.data[0] = backward_distance;
                        msg_goal_dist_angle.data[1] = 0;
                        pub_goal_dist_angle.publish(msg_goal_dist_angle);

                        state = SM_WAITING_FOR_MOVE_BACKWARDS;
                    }
                    else
                    {
                        ROS_INFO("MvnPln.->Robot is Not in static obstacles. Attempting to replan.");
                        state = SM_CALCULATE_PATH;
                    }
                }
                else
                {
                    current_status = publish_status(actionlib_msgs::GoalStatus::ABORTED, goal_id, "Cannot calc path from start to goal", pub_status);
                    //state = SM_INIT;
                    state = SM_CALCULATE_PATH;
                }
                break;
            }

            case SM_WAITING_FOR_MOVE_BACKWARDS:
                if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::SUCCEEDED && simple_move_status_id == -1)
                {
                    simple_move_goal_status.status = 0;
                    ROS_INFO("MvnPln.->Moved backwards succesfully.");
                    state = SM_CALCULATE_PATH;
                }
                else if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::ABORTED)
                {
                    simple_move_goal_status.status = 0;
                    ROS_ERROR("MvnPln.->Simple move reported move aborted. "); //TODO always abort
                                                                               //
                    bool robot_is_in_static_obstacle = false;
                    if (clt_is_in_obstacles.call(srv_check_obstacles) && srv_check_obstacles.response.success)
                    {
                        robot_is_in_static_obstacle = true;
                    }

                    if(robot_is_in_static_obstacle){
                        state = SM_COLLISION_DETECTED;
                    }
                    else{
                        state = SM_CALCULATE_PATH;
                    }
                }
                break;
                
            case SM_CHECK_IF_OBSTACLES:
                if(!clt_are_there_obs.call(srv_check_obstacles) || !srv_check_obstacles.response.success)
                {
                    ROS_ERROR("MvnPln.->No temporal obstacles detected OR service failed. Goal is likely unreachable.");
                    current_status = publish_status(actionlib_msgs::GoalStatus::ABORTED, goal_id,
                                                     "Goal unreachable or in static obstacle (no temporal obs)", pub_status);
                    state = SM_INIT;
                }
                else
                {
                    ROS_WARN("MvnPln.->Temporal obstacles detected. Waiting for them to move.");
                    current_status = publish_status(actionlib_msgs::GoalStatus::ACTIVE, goal_id,
                                                     "Waiting for temporal obstacles to move", pub_status);
                    state = SM_RECOVERY_ROTATE;
                }
                break;

            case SM_WAIT_FOR_NO_OBSTACLES:
                //TODO
                if(!clt_are_there_obs.call(srv_check_obstacles))
                {
                    ROS_ERROR("MvnPln.->Cannot call service for checking temporal obstacles. Announcing failure.");
                    current_status = publish_status(actionlib_msgs::GoalStatus::ABORTED, goal_id,
                                                    "Cannot calculate path from start to goal point", pub_status);
                    state = SM_RECOVERY_ROTATE;
                }
                else if(!srv_check_obstacles.response.success)
                {
                    ROS_INFO("MvnPln.->Temporal obstacles removed. ");
                    state = SM_CALCULATE_PATH;
                }
                else
                {
                    slow_loop.sleep();
                }
                break;
                
            case SM_RECOVERY_ROTATE:
                {
                    //add recovery from cost by r.k 2025/07/04
                    std_msgs::Float32MultiArray rotate_angle;
                    rotate_angle.data.resize(2);
                    rotate_angle.data[0] = 0.0;
                    rotate_angle.data[1] = (2*M_PI)/4;
                    pub_goal_dist_angle.publish(rotate_angle);

                    ROS_WARN("MvnPln.->Recovery from temporary cost.");
                    current_status = publish_status(actionlib_msgs::GoalStatus::ACTIVE, goal_id, "Recovery from temporary cost", pub_status);
                    ros::Duration(1.0).sleep();
       
                    state = SM_CALCULATE_PATH;
                    //state = SM_WAIT_FOR_NO_OBSTACLES;
                }
                break;
            
            case SM_ENABLE_OBS_DETECT:
                msg_bool.data = true;
                pub_obs_detector_enable.publish(msg_bool);
                ROS_INFO("MvnPln.->Obstacle detector enable flag sent. Waiting for obs detector to be enabled..." );
                state = SM_WAIT_FOR_OBS_DETECT;
                break;
                
            case SM_WAIT_FOR_OBS_DETECT:
                ros::topic::waitForMessage<std_msgs::Bool>("/navigation/obs_detector/collision_risk", ros::Duration(100.0));
                ROS_INFO("MvnPln.->Obstacle detector is now available.");
                state = SM_START_MOVE_PATH;
                break;

            case SM_START_MOVE_PATH:
                ROS_INFO("MvnPln.->Starting path following");
                collision_risk = false;
                simple_move_sequencer++;
                path.header.seq = simple_move_sequencer;
                
                //add 2024/09/12
                if (path.poses.size() > 0){
                    pub_goal_path.publish(path);
                    simple_move_goal_status.status = 0;
                    state = SM_WAIT_FOR_MOVE_FINISHED;
                }
                else
                {
                    state = SM_CORRECT_FINAL_ANGLE;
                    //state = SM_FINAL;
                }

                //TODO
                //add for hsrc because sometimes occured self collision //add by r.k 2025/04/23
                pub_tmp_head_pose_cancel.publish(std_msgs::Empty());

                break;

            case SM_WAIT_FOR_MOVE_FINISHED:
                {
                    //motion synth add by r.k 2025/04/10
                    if (ms_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        ROS_INFO_THROTTLE(1.0, "MvnPln.->Arm motion finished successfully.");
                    }
                    //else
                    //{
                    //    ROS_INFO_THROTTLE(1.0, "MvnPln.->Arm motion not completed and not running (state: %s)", ms_ac->getState().toString().c_str());
                    //}

                    //navigation goal
                    //if(error < proximity_criterion && !near_goal_sent)
                    //if(error < proximity_criterion && !near_goal_sent && error > 0.03) //hsrb
                    //if(error < proximity_criterion && !near_goal_sent && error > 0.001) //sim
                    
                    get_robot_position(listener, robot_x, robot_y, robot_a);

                    //add by r.k 2025/06/12 for via
                    //important implementation
                    geometry_msgs::Pose current_pos;
                    current_pos.position.x = robot_x;
                    current_pos.position.y = robot_y;
                    updateViaPoints(current_pos, 0.5); // default threshold 0.5m
                                                       
                    error = sqrt(pow(global_goal.position.x - robot_x, 2) + pow(global_goal.position.y - robot_y, 2)); //important calc dist error

                    ROS_INFO_THROTTLE(1.0, "MvnPln. -> will move error: %f", error);
                    if (error < proximity_criterion && !near_goal_sent)
                    {
                        near_goal_sent = true;
                        ROS_INFO("MvnPln.->Error less than proximity criterion. Sending near goal point status.");
                        publish_status(actionlib_msgs::GoalStatus::ACTIVE, goal_id, "Near goal point", pub_status);
                    }
                    if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::SUCCEEDED && simple_move_status_id == simple_move_sequencer)
                    {
                        simple_move_goal_status.status = 0;
                        ROS_INFO("MvnPln.->Path followed succesfully. ");
                        msg_bool.data = false;
                        pub_obs_detector_enable.publish(msg_bool);
                        state = SM_CORRECT_FINAL_ANGLE;
                    }
                    else if(collision_risk)
                    {
                        //add by r kobayashi 2025/06/27 for disable obs_avoid nearness to goal
                        if (error < disable_backward_threshold)
                        {
                            ROS_WARN("MvnPln.-> Near goal point skip collision risk.");
                            collision_risk = false;
                            break;
                        }


                        ROS_INFO("MvnPln.->COLLISION RISK DETECTED before goal is reached.");
                        if(forcing_backward)
                        {
                            state = SM_COLLISION_DETECTED; //forcing backward
                        }
                        else{
                            state = SM_CALCULATE_PATH;   //default implementation
                        }
                    }
                    else if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::ABORTED && simple_move_status_id == simple_move_sequencer)
                    {
                        // this function can not enter backward
                        simple_move_goal_status.status = 0;
                        ROS_ERROR("MvnPln.->Simple move reported path aborted. Trying again...");
                        
                        //motion synth
                        if (ms_ac->getState() == actionlib::SimpleClientGoalState::ACTIVE)
                        {
                            ms_ac->cancelGoal();
                            ROS_WARN("MvnPln->Canceling arm motion due to move abort.");
                        }
                        state = SM_CALCULATE_PATH;
                    }
                    break;
                }

                //TODO add by r.kobayashi force backward
                case SM_COLLISION_DETECTED:
                    {
                        get_robot_position(listener, robot_x, robot_y, robot_a);

                        if (hypot(robot_x - global_goal.position.x, robot_y - global_goal.position.y) < proximity_criterion){
                            if(near_goal_sent){
                                ROS_WARN("MvnPln.-> Near Goal Reached");
                                state = SM_CORRECT_FINAL_ANGLE;
                            }
                            else{
                                msg_bool.data = false;
                                pub_obs_detector_enable.publish(msg_bool);

                                publish_status(actionlib_msgs::GoalStatus::SUCCEEDED, goal_id, "Goal reached after collision recovery", pub_status);
                                state = SM_INIT;
                            }
                            break;
                        }
                        else{
                            ROS_WARN("MvnPln.->Collision Risk Detected Forcing Backward.");

                            //send backward
                            msg_goal_dist_angle.data[0] = backward_distance; //-0.10 is default
                            msg_goal_dist_angle.data[1] = 0;
                            pub_goal_dist_angle.publish(msg_goal_dist_angle);

                            state = SM_WAITING_FOR_MOVE_BACKWARDS;
                        }
                    }
                    break;
                
                case SM_CORRECT_FINAL_ANGLE:
                {
                    std::cout << "MvnPln.->Correcting final angle." << std::endl;
                    get_robot_position(listener, robot_x, robot_y, robot_a);
                    //error = atan2(global_goal.orientation.z, global_goal.orientation.w)*2 - robot_a;
                    
                    tf::Quaternion q(
                        global_goal.orientation.x,
                        global_goal.orientation.y,
                        global_goal.orientation.z,
                        global_goal.orientation.w
                    );
                    float goal_a = tf::getYaw(q);
                    error = goal_a - robot_a;

                    //if(error  >  M_PI) error -= 2*M_PI;
                    //if(error <= -M_PI) error += 2*M_PI;
                    error = atan2(sin(error), cos(error));
                    msg_goal_dist_angle.data[0] = 0.0;
                    msg_goal_dist_angle.data[1] = error;
                    pub_goal_dist_angle.publish(msg_goal_dist_angle);
                    state = SM_WAIT_FOR_ANGLE_CORRECTED;
                    break;
                }

            case SM_WAIT_FOR_ANGLE_CORRECTED:
                if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::SUCCEEDED && simple_move_status_id == -1)
                {
                    simple_move_goal_status.status = 0;
                    ROS_INFO("MvnPln.->Final angle corrected succesfully.");
                    state = SM_FINAL ;
                }
                else if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::ABORTED)
                {
                    simple_move_goal_status.status = 0;
                    ROS_ERROR("MvnPln.->Simple move reported path aborted. Trying again...");
                    state = SM_CALCULATE_PATH;
                }
                break;
                
            case SM_FINAL:
                
                // motion_synth forcing cancel
                /*
                if (arm_goal_received &&
                    ms_ac->getState() != actionlib::SimpleClientGoalState::SUCCEEDED &&
                    ms_ac->getState() != actionlib::SimpleClientGoalState::ABORTED &&
                    ms_ac->getState() != actionlib::SimpleClientGoalState::PREEMPTED)
                {
                    ROS_ERROR("MvnPln.->Navigation Finished. But arm motion still running, forcing cancel.");
                    ms_ac->cancelGoal();
                }
                */

                /* if (target_arm_pose.has_arm_end_pose) */
                /* { */
                /*     if (ms_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) */
                /*     { */
                /*         ROS_INFO("MvnPln.->Arm motion completed successfully."); */
                /*     } */
                /*     else if(ms_ac->getState().isDone()) */
                /*     { */
                /*         ROS_WARN("MvnPln.->Arm motion completed with state: %s", ms_ac->getState().toString().c_str()); */
                /*     } */
                /*     else */
                /*     { */
                /*         ROS_INFO_THROTTLE(1.0, "MvnPln.->Waiting for arm motion to complete"); */
                /*         break; */
                /*     } */
                /* } */

                std::cout << "MvnPln.->TASK FINISHED." << std::endl;
                publish_status(actionlib_msgs::GoalStatus::SUCCEEDED, goal_id, "Global goal point reached", pub_status);

                //add by r.k 2025/06/12
                via_points.clear();

                state = SM_INIT;
                break;
    
            default:
                std::cout<<"MvnPln.->A VERY STUPID PERSON PROGRAMMED THIS SHIT. APOLOGIES FOR THE INCONVINIENCE. Please contact your dealer."<<std::endl;
        }

        ros::spinOnce();
        loop.sleep();

    }
}

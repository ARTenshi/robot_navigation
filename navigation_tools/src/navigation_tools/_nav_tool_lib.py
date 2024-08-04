#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import rospy
import hsrb_interface
import tf
from std_msgs.msg import Float32MultiArray, Bool, Empty
from geometry_msgs.msg import Pose, Vector3, Quaternion, PoseStamped
from actionlib_msgs.msg import GoalStatus

global globalGoalReached
global goalReached
global omnibase
global pubGlobalGoalXYZ
global whole_body
global robot_stop

globalGoalReached = True
goalReached = True
robot_stop = False


def callback_global_goal_reached(msg):
    global globalGoalReached
    
    globalGoalReached = False
    if msg.status == GoalStatus.SUCCEEDED:    
        globalGoalReached = True


def callback_goal_reached(msg):
    global goalReached
    
    goalReached = False
    if msg.status == GoalStatus.SUCCEEDED:
        goalReached = True


def callback_stop(msg):
    global robot_stop
    robot_stop = True


class nav_module():
    """docstring for nav_module"""

    def __init__(self, select="hsr"):

        global omnibase
        global robot
        global whole_body
        self.navigation_setter = select
        self.goal = Float32MultiArray()
        omnibase = 0
        robot = hsrb_interface.Robot()
        omnibase = robot.get("omni_base")
        whole_body = robot.get("whole_body")
        
        #pubMvnPlnGetCloseXYA   = nh->advertise<geometry_msgs::PoseStamped >("/move_base_simple/goal", 10);
        #pubSimpleMoveDistAngle = nh->advertise<std_msgs::Float32MultiArray>("/simple_move/goal_dist_angle", 10);
        #pubNavigationStop      = nh->advertise<std_msgs::Empty>            ("/navigation/stop", 10);
        
        self.pubGlobalGoalXYZ = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pubMoveRel = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pubDistAngle = rospy.Publisher('/simple_move/goal_dist_angle', Float32MultiArray, queue_size=1)
        self.pubRobotStop = rospy.Publisher('/navigation/stop', Empty, queue_size=1)

        #subNavigationStatus    = nh->subscribe("/navigation/status"       , 10, &JuskeshinoNavigation::callbackNavigationStatus);
        #subSimpleMoveStatus    = nh->subscribe("/simple_move/goal_reached", 10, &JuskeshinoNavigation::callbackSimpleMoveStatus);
        #subNavigationStop      = nh->subscribe("/navigation/stop"         , 10, &JuskeshinoNavigation::callbackNavigationStop);
        #subStop                = nh->subscribe("/stop"                    , 10, &JuskeshinoNavigation::callbackStop);
                
        rospy.Subscriber("/navigation/status", GoalStatus, callback_global_goal_reached)
        rospy.Subscriber("/simple_move/goal_reached", GoalStatus, callback_goal_reached)
        rospy.Subscriber("/stop", Empty, callback_stop)
        rospy.Subscriber("/global_pose", PoseStamped, self.global_pose_callback)
        
        rospy.sleep(1.0) #DO NOT DELET THIS CODE, IMPORTANT
        
        self.set_navigation_type(select)

        self.global_pose = PoseStamped()

    def set_navigation_type(self, type_nav):
        if type_nav == "hsr" or type_nav == "pumas":
            self.navigation_setter = type_nav
        else:
            print("Invalid navigation mode")
        print("USING " + str(self.navigation_setter).upper() + " NAVIGATION BY DEFAULT")

    def get_navigation_type(self):
        return self.navigation_setter

    def global_pose_callback(self, msg):
        self.global_pose = msg

    def getClose(self, x, y, yaw, timeout, goal_distance=None):
        global globalGoalReached
        global robot_stop
        rate = rospy.Rate(10)
        
        #goal = Float32MultiArray()
        #goal.data = [x, y, yaw]
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0

        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        
        globalGoalReached = False
        robot_stop = False
        if timeout != 0:
            attemps = int(timeout*10)
        else:
            attemps = 10000000000000
        # print  goal.data
        
        self.pubGlobalGoalXYZ.publish(goal)
        rate.sleep()
        rospy.sleep(5.)
        while not globalGoalReached and not rospy.is_shutdown() and not robot_stop and attemps >= 0:

            if goal_distance:
                now_x, now_y = self.global_pose.pose.position.x, self.global_pose.pose.position.y
                now_distance = math.sqrt((x - now_x) ** 2 + (y - now_y) ** 2)
                #print(globalGoalReached, robot_stop, attemps, now_distance)
                if now_distance < goal_distance:
                    break
            #else:
            #    print(globalGoalReached, robot_stop, attemps) #TODO

            attemps -= 1
            rate.sleep()
        robot_stop = False
        if not globalGoalReached:
            msg_stop = Empty()
            self.pubRobotStop.publish(msg_stop)
            rate.sleep()
            rospy.sleep(5.)
        x = rospy.Duration.from_sec(2.5)
        rospy.sleep(x)

    def go_abs(self, x, y, theta, timeout=0.0, type_nav=None, goal_distance=None):
        print("Call ABS mode in pumas nav")
        if type_nav == "pumas":
            self.getClose(x, y, theta, timeout, goal_distance)
        elif type_nav == "hsr":
            omnibase.go_abs(x, y, theta, timeout)
        else:
            if self.navigation_setter == "pumas":
                self.getClose(x, y, theta, timeout)
            else:
                omnibase.go_abs(x, y, theta, timeout)

    def moveDistAngle(self, x, yaw, timeout):
        global goalReached
        global robot_stop
        rate = rospy.Rate(10)
        goal = Float32MultiArray()
        goalReached = False
        robot_stop = False
        goal.data = [x, yaw]
        # print  goal.data
        if timeout != 0:
            attemps = int(timeout*10)
        else:
            attemps = 10000000000000
        self.pubDistAngle.publish(goal)
        rate.sleep()
        rospy.sleep(5.)
        while not goalReached and not rospy.is_shutdown() and not robot_stop and attemps >= 0:
            attemps -= 1
            rate.sleep()
        robot_stop = False
        if not goalReached:
            msg_stop = Empty()
            self.pubRobotStop.publish(msg_stop)
            rate.sleep()
            rospy.sleep(5.)
        rate.sleep()
        x = rospy.Duration.from_sec(2.5)
        rospy.sleep(x)

    def go_dist_angle(self, x=0.0, yaw=0.0, timeout=0.0, type_nav=None):
        self.moveDistAngle(x, yaw, timeout)

    def moveRel(self, x, y, yaw, timeout):
        global globalGoalReached
        global robot_stop
        rate = rospy.Rate(10)
        globalGoalReached = False
        robot_stop = False
        
        #goal = Float32MultiArray()
        #goal.data = [x, y, yaw]
        
        goal = PoseStamped()
        goal.header.frame_id = "base_link"
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0

        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        
        # print  goal.data
        if timeout != 0:
            attemps = int(timeout*10)
        else:
            attemps = 10000000000000
        self.pubMoveRel.publish(goal)
        rate.sleep()
        rospy.sleep(5.)
        while not globalGoalReached and not rospy.is_shutdown() and not robot_stop and attemps >= 0:
            attemps -= 1
            rate.sleep()
        robot_stop = False
        if not globalGoalReached:
            msg_stop = Empty()
            self.pubRobotStop.publish(msg_stop)
            rate.sleep()
            rospy.sleep(5.)
        rate.sleep()
        x = rospy.Duration.from_sec(2.5)
        rospy.sleep(x)

    def go_rel(self, x=0.0, y=0.0, yaw=0.0, timeout=0.0, type_nav=None):
        print("Call REL mode in pumas nav")
        if type_nav == "pumas":
            self.moveRel(x, y, yaw, timeout)
        elif type_nav == "hsr":
            omnibase.go_rel(x, y, yaw, timeout)
        else:
            if self.navigation_setter == "pumas":
                self.moveRel(x, y, yaw, timeout)
            else:
                omnibase.go_rel(x, y, yaw, timeout)

    ##############################
    ##   HSR Functions bypass   ##
    ##############################

    def cancel_goal(self):
        omnibase.cancel_goal()

    def create_follow_trajectory_goal(self, poses, time_from_starts=[], ref_frame_id=None):
        return omnibase.create_follow_trajectory_goal(poses, time_from_starts, ref_frame_id)

    def create_go_pose_goal(self, pose, ref_frame_id=None):
        return omnibase.create_go_pose_goal(pose, ref_frame_id)

    def execute(self, goal):
        omnibase.execute(goal)

    def follow_trajectory(self, poses, time_from_starts=[], ref_frame_id=None):
        omnibase.follow_trajectory(poses, time_from_starts, ref_frame_id)

    def get_pose(self, ref_frame_id=None):
        return omnibase.get_pose(ref_frame_id)

    def go(self,  x, y, yaw, timeout=0.0, relative=False):
        omnibase.go(x, y, yaw, timeout, relative)

    def go_pose(self, pose=Pose(Vector3(x=0.0, y=0.0, z=0.0), Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)), timeout=0.0, ref_frame_id=None):
        omnibase.go_pose(pose, timeout, ref_frame_id)

    def is_moving(self):
        return omnibase.is_moving()

    def is_succeeded(self):
        return omnibase.is_succeeded()

    def move(self, pose, timeout=0.0, ref_frame_id=None):
        omnibase.move(pose, timeout, ref_frame_id)

    def pose(self):
        return omnibase.pose

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
import hsrb_interface
import tf

from std_msgs.msg import Float32MultiArray, Bool, Empty
from geometry_msgs.msg import Pose, Vector3, Quaternion, PoseStamped, Pose2D
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker

from std_srvs.srv import Trigger

import numpy as np
import random

#from navigation_tools.search_safe_point import SearchSafePoint

class NavModule:
    """docstring for NavModule"""

    def __init__(self, select="hsr"):

        self.global_goal_reached = True
        self.goal_reached = True
        self.robot_stop = False

        self.robot = hsrb_interface.Robot()
        self.omnibase = self.robot.get("omni_base")
        self.whole_body = self.robot.get("whole_body")

        #for safe_point
        #self.search_safe_point = SearchSafePoint()
        #self.goal_torelance = 0
        #self.refer_size = 2
        #self.replan_radius = 0.1
        self.distance_from_cost = 0.1
        self.diagonal_devided_circle = 32 #how many diagonal devided circle
        self.points = []
        self.marker_num = 0

        #publishers
        self.pub_global_goal_xyz = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_move_rel = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_dist_angle = rospy.Publisher('/simple_move/goal_dist_angle', Float32MultiArray, queue_size=1)
        self.pub_robot_stop = rospy.Publisher('/navigation/stop', Empty, queue_size=1)

        self.pub_marker = rospy.Publisher('/nav_goal_marker', Marker, queue_size=10)

        #for lib env manage
        #self.pub_robot_stop = rospy.Publisher('/lme/first_rotate', Bool, queue_size=1)

        rospy.Subscriber("/navigation/status", GoalStatus, self.callback_global_goal_reached)
        rospy.Subscriber("/simple_move/goal_reached", GoalStatus, self.callback_goal_reached)
        rospy.Subscriber("/stop", Empty, self.callback_stop)
        rospy.Subscriber("/global_pose", PoseStamped, self.global_pose_callback)

        #obstacle detector's srv
        self.is_inside_obstacles = rospy.ServiceProxy('/map_augmenter/is_inside_obstacles', Trigger)

        self.are_there_obstacles = rospy.ServiceProxy('/map_augmenter/are_there_obstacles', Trigger)

        rospy.sleep(1.0)  # DO NOT DELETE THIS CODE, IMPORTANT

        self.set_navigation_type(select)
        self.global_pose = PoseStamped()
        self.global_goal_xyz = PoseStamped() 



    def callback_global_goal_reached(self, msg):
        self.global_goal_reached = False
        #service response
        #self.res_is_inside = self.is_inside_obstacles()
        #self.res_are_there = self.are_there_obstacles()

        if msg.status == GoalStatus.SUCCEEDED:
            self.global_goal_reached = True
            #TODO return 'success'

        if msg.status == GoalStatus.ACTIVE:
            if msg.text == 'Waiting for temporal obstacles to move':
                rospy.loginfo('NavigationStatus -> Waiting for temporal obstacle to move')
                rospy.loginfo('NavigationStatus -> Replanning by search_safe_point')

                #self.recovory_from_cost()
                self.replan_safe_point()
                self.distance_from_cost += 0.1

        if msg.status == GoalStatus.ABORTED:
            if msg.text == 'Cannot calculate path from start to goal point':
                #rospy.loginfo('NavigationStatus -> Cannot calculate path from start to goal point')
                #if self.res_is_inside.success:
                #    rospy.logwarn('NavModuleStatus -> is_inside_obstacles, recovery_from_cost')
                    #TODO turn or backwards 
                    #lme first rotate

                #elif self.res_are_there.success:
                #    rospy.logwarn('NavModuleStatus -> are_there_obstacles, replan_safe_point')
                if len(points) == 0:
                    self.replan_safe_point()
                    self.distance_from_cost += 0.1
                elif len(points) != 0:


                #elif (self.res_are_there.success == False and self.res_is_inside.success == False):
                #    pass

                #else:
                #    rospy.logerr('NavModuleStatus -> ABORTED, Failed')

                
            if msg.text == 'Cancelling current movement':
                rospy.loginfo('NavigationStatus -> Cancelling current movement')
                #rospy.loginfo('NavigationStatus -> Replanning by search_safe_point')
                #self.replan_safe_point()

    def callback_goal_reached(self, msg):
        self.goal_reached = False
        if msg.status == GoalStatus.SUCCEEDED:
            self.goal_reached = True
            self.distance_from_cost = 0.1

    def callback_stop(self, msg):
        self.robot_stop = True
        self.distance_from_cost = 0.1

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

    def pose_stamped2pose_2d(self, pose_stamped):
        pose2d = Pose2D()
        pose2d.x = pose_stamped.pose.position.x
        pose2d.y = pose_stamped.pose.position.y
        orientation = pose_stamped.pose.orientation
        euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        pose2d.theta = euler[2]  #euler[2] is yaw
        return pose2d

    def recovery_from_cost(self):
        rospy.loginfo('called recovery_from_cost')
        rospy.loginfo(self.global_pose)

    def replan_safe_point(self):
        rospy.loginfo('called replan_safe_point')

        current_goal_pose2d = self.pose_stamped2pose_2d(self.global_goal_xyz)
        radius = self.distance_from_cost
        num_points = 16
        points = self.diagonal_safe_points(current_goal_pose2d, radius, num_points)

        for safe_point in points:
            goal = PoseStamped()
            goal.header.frame_id = self.global_pose.header.frame_id
            goal.pose.position.x = safe_point.x
            goal.pose.position.y = safe_point.y
            q = tf.transformations.quaternion_from_euler(0, 0, safe_point.theta)
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]

            self.marker_plot(goal)
            self.pub_global_goal_xyz.publish(goal)

        rospy.loginfo('called replan_safe_point -> but not found')

    def diagonal_safe_points(self, center_pose2d, radius, num_points):
        self.points = []
        angle_increment = 2 * math.pi / num_points

        for i in range(num_points):
            angle = i * angle_increment
            x = center_pose2d.x + radius * math.cos(angle)
            y = center_pose2d.y + radius * math.sin(angle)
            points.append(Pose2D(x, y, angle))

        return self.points


    """
    def replan_safe_point(self):
        rospy.loginfo('called replan_safe_point')

        ###simple method for avoiding error, search safe point by using 
        current_goal_pose2d = self.pose_stamped2pose_2d(self.global_goal_xyz) #Pose2D format

        angle_increment = 2 * math.pi / self.diagonal_devided_circle

        for i in range(self.diagonal_devided_circle):
            angle = i * angle_increment
            #calc_goal_x = current_goal_pose2d.x - self.replan_radius * math.sin(current_goal_pose2d.theta)
            #calc_goal_y = current_goal_pose2d.y - self.replan_radius * math.cos(current_goal_pose2d.theta)

            calc_goal_x = current_goal_pose2d.x - self.distance_from_cost * math.cos(angle)
            calc_goal_y = current_goal_pose2d.y - self.distance_from_cost * math.sin(angle)
            safe_point = Pose2D(calc_goal_x, calc_goal_y, angle)

            goal = PoseStamped()
            goal.header.frame_id = self.global_pose.header.frame_id
            goal.pose.position.x = safe_point.x
            goal.pose.position.y = safe_point.y
            q = tf.transformations.quaternion_from_euler(0, 0, safe_point.theta)
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]

            self.marker_plot(goal)

            self.pub_global_goal_xyz.publish(goal)

        rospy.loginfo('called replan_safe_point -> but not found')
        #self.distance_from_cost += 0.2

        


        ### high level method for avoiding error, search safe point by using gridmap
        #safe_point = self.search_safe_point.get_most_safe_point(goal_pose2d, self.goal_torelance, self.refer_size)

        #if safe_point:
        #    rospy.loginfo(safe_point)
        #    goal = PoseStamped()
        #    goal.header.frame_id = self.global_pose.header.frame_id
        #    goal.pose.position.x = safe_point.x
        #    goal.pose.position.y = safe_point.y
        #    q = tf.transformations.quaternion_from_euler(0, 0, safe_point.theta)
        #    goal.pose.orientation.x = q[0]
        #    goal.pose.orientation.y = q[1]
        #    goal.pose.orientation.z = q[2]
        #    goal.pose.orientation.w = q[3]
        #    self.pub_global_goal_xyz.publish(goal)
        #else:
        #    #TODO: increase goal torelance
        #    self.goal_torelance += 1  #grid
        #    #self.refer_size += 1     #grid
    """
    
    def marker_plot(self, goal):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_markers"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = goal.pose.position.x
        marker.pose.position.y = goal.pose.position.y
        marker.pose.position.z = 0
        marker.pose.orientation.x = goal.pose.orientation.x
        marker.pose.orientation.y = goal.pose.orientation.y
        marker.pose.orientation.z = goal.pose.orientation.z
        marker.pose.orientation.w = goal.pose.orientation.w
        marker.scale.x = 0.1  
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0  
        marker.color.r = 1.0  
        marker.color.g = 0.0 
        marker.color.b = 0.0
        self.pub_marker.publish(marker)

    def get_close(self, x, y, yaw, timeout, goal_distance=None):
        rate = rospy.Rate(10)

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

        self.global_goal_reached = False
        self.robot_stop = False
        if timeout != 0:
            attempts = int(timeout * 10)
        else:
            attempts = float('inf')
        
        #important: for replan_safe_point 
        self.global_goal_xyz = goal

        self.pub_global_goal_xyz.publish(goal)
        rate.sleep()
        rospy.sleep(5.0)

        while not self.global_goal_reached and not rospy.is_shutdown() and not self.robot_stop and attempts >= 0:
            if goal_distance:
                now_x, now_y = self.global_pose.pose.position.x, self.global_pose.pose.position.y
                now_distance = math.sqrt((x - now_x) ** 2 + (y - now_y) ** 2)
                if now_distance < goal_distance:
                    break

            attempts -= 1
            rate.sleep()

        self.robot_stop = False
        if not self.global_goal_reached:
            msg_stop = Empty()
            self.pub_robot_stop.publish(msg_stop)
            rate.sleep()
            rospy.sleep(5.0)
        rospy.sleep(rospy.Duration.from_sec(2.5))

    def go_abs(self, x, y, theta, timeout=0.0, type_nav=None, goal_distance=None):
        print("Call ABS mode in pumas nav")
        if type_nav == "pumas":
            self.get_close(x, y, theta, timeout, goal_distance)
        elif type_nav == "hsr":
            self.omnibase.go_abs(x, y, theta, timeout)
        else:
            if self.navigation_setter == "pumas":
                self.get_close(x, y, theta, timeout)
            else:
                self.omnibase.go_abs(x, y, theta, timeout)

    def move_dist_angle(self, x, yaw, timeout):
        rate = rospy.Rate(10)
        goal = Float32MultiArray()
        self.goal_reached = False
        self.robot_stop = False
        goal.data = [x, yaw]

        if timeout != 0:
            attempts = int(timeout * 10)
        else:
            attempts = float('inf')

        self.pub_dist_angle.publish(goal)
        rate.sleep()
        rospy.sleep(5.0)

        while not self.goal_reached and not rospy.is_shutdown() and not self.robot_stop and attempts >= 0:
            attempts -= 1
            rate.sleep()

        self.robot_stop = False
        if not self.goal_reached:
            msg_stop = Empty()
            self.pub_robot_stop.publish(msg_stop)
            rate.sleep()
            rospy.sleep(5.0)
        rospy.sleep(rospy.Duration.from_sec(2.5))

    def go_dist_angle(self, x=0.0, yaw=0.0, timeout=0.0, type_nav=None):
        self.move_dist_angle(x, yaw, timeout)

    def move_rel(self, x, y, yaw, timeout):
        rate = rospy.Rate(10)
        self.global_goal_reached = False
        self.robot_stop = False

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

        if timeout != 0:
            attempts = int(timeout * 10)
        else:
            attempts = float('inf')

        self.pub_move_rel.publish(goal)
        rate.sleep()
        rospy.sleep(5.0)

        while not self.global_goal_reached and not rospy.is_shutdown() and not self.robot_stop and attempts >= 0:
            attempts -= 1
            rate.sleep()

        self.robot_stop = False
        if not self.global_goal_reached:
            msg_stop = Empty()
            self.pub_robot_stop.publish(msg_stop)
            rate.sleep()
            rospy.sleep(5.0)
        rospy.sleep(rospy.Duration.from_sec(2.5))

    def go_rel(self, x=0.0, y=0.0, yaw=0.0, timeout=0.0, type_nav=None):
        print("Call REL mode in pumas nav")
        if type_nav == "pumas":
            self.move_rel(x, y, yaw, timeout)
        elif type_nav == "hsr":
            self.omnibase.go_rel(x, y, yaw, timeout)
        else:
            if self.navigation_setter == "pumas":
                self.move_rel(x, y, yaw, timeout)
            else:
                self.omnibase.go_rel(x, y, yaw, timeout)

    ##############################
    ##   HSR Functions bypass   ##
    ##############################

    def cancel_goal(self):
        self.omnibase.cancel_goal()
    def create_follow_trajectory_goal(self, poses, time_from_starts=[], ref_frame_id=None):
        return self.omnibase.create_follow_trajectory_goal(poses, time_from_starts, ref_frame_id)

    def create_go_pose_goal(self, pose, ref_frame_id=None):
        return self.omnibase.create_go_pose_goal(pose, ref_frame_id)

    def execute(self, goal):
        self.omnibase.execute(goal)

    def follow_trajectory(self, poses, time_from_starts=[], ref_frame_id=None):
        self.omnibase.follow_trajectory(poses, time_from_starts, ref_frame_id)

    def get_pose(self, ref_frame_id=None):
        return self.omnibase.get_pose(ref_frame_id)

    def go(self, x, y, yaw, timeout=0.0, relative=False):
        self.omnibase.go(x, y, yaw, timeout, relative)

    def go_pose(self, pose=Pose(Vector3(x=0.0, y=0.0, z=0.0), Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)), timeout=0.0, ref_frame_id=None):
        self.omnibase.go_pose(pose, timeout, ref_frame_id)

    def is_moving(self):
        return self.omnibase.is_moving()

    def is_succeeded(self):
        return self.omnibase.is_succeeded()

    def move(self, pose, timeout=0.0, ref_frame_id=None):
        self.omnibase.move(pose, timeout, ref_frame_id)

    def pose(self):
        return self.omnibase.pose

if __name__ == "__main__":
    rospy.init_node('navigation_module')
    nav = NavModule(select="hsr")

    #example
    #nav.go_abs(3.68, -1.65, 0, 0, 'pumas')
    nav.go_abs(4.78, -0.32, -1.57, 0, 'pumas')


#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import math
import numpy as np
from typing import Union # go_nav function
from fractions import Fraction

import rospy

from hsrlib.hsrif import HSRInterfaces
from hsrlib.rosif import ROSInterfaces

import tf

from std_msgs.msg import Float32MultiArray, Bool, Empty
from nav_msgs.msg import Path

from geometry_msgs.msg import Pose, Vector3, Quaternion, PoseStamped, Pose2D, Twist
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger



class NavModule:
    """Navigation Module for the robot"""
    __instance = None

    def __new__(cls, *args, **kargs):
        """シングルトン化処理."""
        if cls.__instance is None:
            cls.__instance = super(NavModule, cls).__new__(cls)
            cls.__initialized = False
        return cls.__instance

    def __init__(self, select="hsr"):

        if self.__initialized:
            rospy.logwarn("NavModule -> called class initializaion")
            return
        self.global_goal_reached = True
        self.goal_reached = True
        self.robot_stop = False

        self.hsrif = HSRInterfaces()
        self.rosif = ROSInterfaces()
        
        # 8 diagonal safe point detection
        self.distance_from_cost = 0.1
        self.marker_num = 0
        self.points = []

        self.global_pose = None
        self.marker = Marker()

        self.path_plan_client = rospy.ServiceProxy('/path_planner/path_plan_status', Trigger)
        self.pub_global_goal_xyz = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_move_rel = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_dist_angle = rospy.Publisher('/simple_move/goal_dist_angle', Float32MultiArray, queue_size=1)
        self.pub_robot_stop = rospy.Publisher('/navigation/stop', Empty, queue_size=1)

        self.pub_cmd_vel = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
        self.pub_marker = rospy.Publisher('/nav_goal_marker', Marker, queue_size=10)

        rospy.Subscriber("/navigation/status", GoalStatus, self.callback_global_goal_reached)
        rospy.Subscriber("/simple_move/goal_reached", GoalStatus, self.callback_goal_reached)
        rospy.Subscriber("/stop", Empty, self.callback_stop)
        rospy.Subscriber("/global_pose", PoseStamped, self.global_pose_callback)

        # for obstacle detection on/off
        rospy.get_param('/obs_detector/use_lidar')
        rospy.get_param('/obs_detector/use_point_cloud')

        self.is_inside_obstacles = rospy.ServiceProxy('/map_augmenter/is_inside_obstacles', Trigger)
        self.are_there_obstacles = rospy.ServiceProxy('/map_augmenter/are_there_obstacles', Trigger)

        rospy.sleep(1.0)

        self.set_navigation_type(select)

    def callback_global_goal_reached(self, msg):

        path_plan_success = self.path_plan_client()
        if path_plan_success.success:
            rospy.loginfo("NavModule -> pathplan success")
            self.points = []
            self.distance_from_cost = 0.1
            #rospy.logerr(self.points)
        else:
            rospy.loginfo("NavModule -> pathplan fail")

        self.global_goal_reached = False
        if msg.status == GoalStatus.SUCCEEDED:
            self.global_goal_reached = True

        if msg.status == GoalStatus.ACTIVE:
            if msg.text == 'Waiting for temporal obstacles to move':
                rospy.loginfo('NavigationStatus -> Waiting for temporal obstacle to move')

                #TODO temporal obstacles
                #self.go_rel(x=0.2, y=0.0, yaw=0.0, timeout=0.0, type_nav="hsr")
                #self.go_rel(x=-0.2, y=0.0, yaw=0.0, timeout=0.0, type_nav="hsr")
                self.recovery_from_cost()

                
                ##if len(self.points) == 0:
                #if not self.points:
                #   self.replan_safe_point()
                #   self.distance_from_cost += 0.2
                #else:
                #   self.publish_next_point()

        elif msg.status == GoalStatus.ABORTED:
            #TODO start incollision
            if msg.text == 'Cannot calculate path from start to goal point':
                rospy.loginfo('NavigationStatus -> Cannot calculate path from start to goal point')

                #state is goal incollision
                #if not self.points:
                #    self.replan_safe_point()
                #    self.distance_from_cost += 0.2
                #else:
                #    self.publish_next_point()

            elif msg.text == 'Cancelling current movement':
                rospy.loginfo('NavigationStatus -> Cancelling current movement')

    def callback_goal_reached(self, msg):
        self.goal_reached = False
        if msg.status == GoalStatus.SUCCEEDED:
            self.goal_reached = True
            self.distance_from_cost = 0.1

    def callback_stop(self, msg):
        self.robot_stop = True
        self.distance_from_cost = 0.1

    def set_navigation_type(self, type_nav):
        valid_types = ["hsr", "pumas"]
        if type_nav in valid_types:
            self.navigation_setter = type_nav
        else:
            rospy.logerr("Invalid navigation mode")
        rospy.loginfo(f"USING {self.navigation_setter.upper()} NAVIGATION BY DEFAULT")


    def global_pose_callback(self, msg):
        self.global_pose = msg

    def pose_stamped2pose_2d(self, pose_stamped):
        pose2d = Pose2D()
        pose2d.x = pose_stamped.pose.position.x
        pose2d.y = pose_stamped.pose.position.y
        orientation = pose_stamped.pose.orientation
        euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        pose2d.theta = euler[2]
        return pose2d

    def replan_safe_point(self):
        rospy.loginfo('called replan_safe_point')
        try:
            #rospy.logerr(f"self.global_goal_xyz: { self.global_goal_xyz}")
            current_goal_pose2d = self.pose_stamped2pose_2d(self.global_goal_xyz)
            radius = self.distance_from_cost
            num_points = 8
            self.points = self.calculate_safe_points(current_goal_pose2d, radius, num_points)

            if self.points:
                self.publish_next_point()
            else:
                #self.hsrif.tts.say('Path Planning failed. I will try again.', language='en', queue=True, sync=True)
                rospy.loginfo('called replan_safe_point -> but not found')
                #self.distance_from_cost += 0.2

        except:
            import traceback
            traceback.print_exc()


    def publish_next_point(self):
        rospy.loginfo("called publish next point")

        safe_point = self.points.pop(0)  # Remove the first value
        goal = PoseStamped()
        goal.header.frame_id = self.global_goal_xyz.header.frame_id
        goal.pose.position.x = safe_point.x
        goal.pose.position.y = safe_point.y


        # robot will rotate original goal rotation
        goal_direction = math.atan2(self.global_goal_xyz.pose.position.y - safe_point.y,
                                    self.global_goal_xyz.pose.position.x - safe_point.x)

        q = tf.transformations.quaternion_from_euler(0, 0, goal_direction)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        self.send_goal(goal)

    def calculate_safe_points(self, center_pose2d, radius, num_points):
        points = []
        angle_increment = 2 * math.pi / (num_points - 1)

        for i in range(num_points):
            angle = i * angle_increment
            x = center_pose2d.x + radius * math.cos(angle)
            y = center_pose2d.y + radius * math.sin(angle)
            points.append(Pose2D(x, y, angle))

        return points

    def marker_plot(self, goal):
        
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "goal_markers"
        self.marker.id = self.marker_num
        self.marker_num += 1
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose = goal.pose
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.pub_marker.publish(self.marker)

    def recovery_from_cost(self):

        rospy.loginfo('NavigationStatus. -> Recovery From Cost -> Rotation')
        #self.hsrif.tts.say('Obstacle detected.', language='en')

        current_orientation = self.global_pose.pose.orientation
        _, _, current_yaw = tf.transformations.euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])
        opposite_yaw = current_yaw + math.pi

        twist = Twist()
        twist.angular.z = 0.5 if opposite_yaw > current_yaw else -0.5
        rate = rospy.Rate(10)
        duration = abs(opposite_yaw - current_yaw) / 0.5

        for _ in range(int(duration * 10)):  # duration in seconds, rate is 10 Hz
            self.pub_cmd_vel.publish(twist)
            rate.sleep()

        # Stop rotation
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)
        

    def get_close(self, x, y, yaw, timeout, goal_distance=None):
        rate = rospy.Rate(10)
        goal = self.create_goal_pose(x, y, yaw, "map")

        self.global_goal_reached = False
        self.robot_stop = False
        attempts = int(timeout * 10) if timeout != 0 else float('inf')

        self.global_goal_xyz = copy.deepcopy(goal)
        #print("get_close self.global_goal_xyz: ", self.global_goal_xyz)
        #self.pub_global_goal_xyz.publish(goal)
        self.send_goal(goal)
        #rospy.sleep(1.0)

        while not self.global_goal_reached and not rospy.is_shutdown() and not self.robot_stop and attempts >= 0:
            if goal_distance:
                now_x, now_y = self.global_pose.pose.position.x, self.global_pose.pose.position.y
                now_distance = math.sqrt((x - now_x) ** 2 + (y - now_y) ** 2)
                if now_distance < goal_distance:
                    break

            attempts -= 1
            rate.sleep()

        self.handle_robot_stop()

    def go_abs(self, x, y, theta, timeout=0, type_nav=None, goal_distance=None):
        if type_nav == "pumas":
            rospy.loginfo("Call ABS mode in pumas nav")
            self.get_close(x, y, theta, timeout, goal_distance)

        elif type_nav == "hsr":
            rospy.loginfo("Call ABS mode in hsr nav")
            self.hsrif.omni_base.go_abs(x, y, theta, timeout)

        else:
            if self.navigation_setter == "pumas":
                rospy.loginfo("Call ABS mode in pumas nav")
                self.get_close(x, y, theta, timeout)

            else:
                rospy.loginfo("Call ABS mode in hsr nav")
                self.hsrif.omni_base.go_abs(x, y, theta, timeout)

    

    def send_goal(self, goal):
        rospy.logwarn('NavModule -> sending new goal')
        #rospy.logwarn(goal)
        self.marker_plot(goal)
        self.pub_global_goal_xyz.publish(goal)


    def move_dist_angle(self, x, yaw, timeout):
        rate = rospy.Rate(10)
        goal = Float32MultiArray()

        self.goal_reached = False
        self.robot_stop = False
        goal.data = [x, yaw]

        attempts = int(timeout * 10) if timeout != 0 else float('inf')

        self.pub_dist_angle.publish(goal)
        rospy.sleep(1.0)

        while not self.goal_reached and not rospy.is_shutdown() and not self.robot_stop and attempts >= 0:
            attempts -= 1
            rate.sleep()

        self.handle_robot_stop()

    def go_dist_angle(self, x=0.0, yaw=0.0, timeout=0.0, type_nav=None):
        self.move_dist_angle(x, yaw, timeout)

    def move_rel(self, x, y, yaw, timeout):

        rate = rospy.Rate(10)
        self.global_goal_reached = False
        self.robot_stop = False

        goal = self.create_goal_pose(x, y, yaw, "base_link")

        attempts = int(timeout * 10) if timeout != 0 else float('inf')
    
        self.pub_move_rel.publish(goal)
        rospy.sleep(1.0)

        while not self.global_goal_reached and not rospy.is_shutdown() and not self.robot_stop and attempts >= 0:
            attempts -= 1
            rate.sleep()

        self.handle_robot_stop()

    def go_rel(self, x=0.0, y=0.0, yaw=0.0, timeout=0.0, type_nav=None):
        #in the tmc simulator, if rel mode is not moving, please set the map.yaml and map.pgm in the ~/.ros/maps/

        if type_nav == "pumas":
            rospy.loginfo("Call REL mode in pumas nav")
            self.move_rel(x, y, yaw, timeout)

        elif type_nav == "hsr":
            rospy.loginfo("Call REL mode in hsr nav")
            self.hsrif.omni_base.go_rel(x, y, yaw, timeout)

        else:

            if self.navigation_setter == "pumas":
                rospy.loginfo("Call REL mode in pumas nav")
                self.move_rel(x, y, yaw, timeout)
            else:
                rospy.loginfo("Call REL mode in hsr nav")
                self.hsrif.omni_base.go_rel(x, y, yaw, timeout)

    def create_goal_pose(self, x, y, yaw, frame_id):
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        return goal

    def handle_robot_stop(self):
        if not self.global_goal_reached:
            msg_stop = Empty()
            self.pub_robot_stop.publish(msg_stop)
            rospy.sleep(1.0)
        rospy.sleep(1.0)

    def rotate_yaw(self, goal_pose):

        pi = 3.1415926535

        #current_pose = self.pose_stamped2pose_2d(self.global_pose) #TODO 小数点レベルで誤差あり
        current_pose = self.pose()
        x, y, theta = current_pose.x, current_pose.y, current_pose.theta

        if (abs(theta) > pi/2.000) and (abs(goal_pose.theta) > pi/2.000):

            if theta > pi/2.000:
                theta -=  pi
            else:
                theta +=  pi

            if goal.theta > pi/2.000:
                goal_pose.theta -=  pi
            else:
                goal_pose.theta +=  pi

        self.rosif.pub.command_velocity_in_sec(0.0, 0.0, goal_pose.theta - theta , 1.0)


    def use_obstacle_detection(self, status):

        rospy.set_param('/obs_detector/use_lidar', status)
        rospy.set_param('/obs_detector/use_point_cloud', status)
        rospy.logwarn(f"NavModule.-> obstacle_detection use LIDAR >>  {status}")
        rospy.logwarn(f"NavModule.-> obstacle_detection use POINT CLOUD >>  {status}")


    #########################################
    ##   HSR Functions bypass with hsrif   ##
    #########################################

    def cancel_goal(self):
        self.hsrif.omni_base.cancel_goal()

    def create_follow_trajectory_goal(self, poses, time_from_starts=[], ref_frame_id=None):
        return self.hsrif.omni_base.create_follow_trajectory_goal(poses, time_from_starts, ref_frame_id)

    def create_go_pose_goal(self, pose, ref_frame_id=None):
        return self.hsrif.omni_base.create_go_pose_goal(pose, ref_frame_id)

    def execute(self, goal):
        self.hsrif.omni_base.execute(goal)

    def follow_trajectory(self, poses, time_from_starts=[], ref_frame_id=None):
        self.hsrif.omni_base.follow_trajectory(poses, time_from_starts, ref_frame_id)

    def get_pose(self, ref_frame_id=None):
        return self.hsrif.omni_base.get_pose(ref_frame_id)

    def go(self, x, y, yaw, timeout=0.0, relative=False):
        self.hsrif.omni_base.go(x, y, yaw, timeout, relative)

    def go_pose(self, pose=Pose(Vector3(x=0.0, y=0.0, z=0.0), Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)), timeout=0.0, ref_frame_id=None):
        
        self.hsrif.omni_base.go_pose(pose, timeout, ref_frame_id)

    def is_moving(self):
        return self.hsrif.omni_base.is_moving()

    def is_succeeded(self):
        return self.hsrif.omni_base.is_succeeded()

    def move(self, pose, timeout=0.0, ref_frame_id=None):
        self.hsrif.omni_base.move(pose, timeout, ref_frame_id)

    def pose(self):
        return self.hsrif.omni_base.get_pose()

    #######################
    ##   call function   ##
    #######################
    def nav_goal(self, goal: Union[Pose2D, str], pose = None, nav_type = "pumas", nav_mode = "abs", nav_timeout = 0, goal_distance = 0.0, angle_correction=True, obstacle_detection=True):
         """ _NavModulePumas_
         Args:
         goal (Pose2D): Final Position given by x,y,yaw
         Pose (Dict): Final Pose <<<<<< under construction >>>>>>>
         nav_type(pumas_nav) (str): pumas_nav -> "pumas"(default) , toyota_nav -> "hsr"
         nav_mode(pumas_nav) (str): "abs" or "rel", default -> abs
         nav_timeout(pumas_nav) (Float):50.0 -> 50s, 0 -> infinity
         goal_distance(pumas_nav, only abs mode) (Float): goal position - goal_distance
         """
         rospy.logerr(goal)

         # under constructoin: obstacle_detection (lidar and point_cloud) can set to only on/off. 
         if obstacle_detection:
             self.use_obstacle_detection(status=True)
         else:
             self.use_obstacle_detection(status=False)

         if nav_mode == "rel":
             self.go_rel(goal.x, goal.y, goal.theta, nav_timeout, nav_type)
         else:
             self.go_abs(goal.x, goal.y, goal.theta, nav_timeout, nav_type, goal_distance)

         if angle_correction is True:
             self.rotate_yaw(goal)
         else:
             pass



if __name__ == "__main__":
    rospy.init_node('navigation_module')
    nav = NavModule(select="pumas")

    # example usage
    #nav.go_rel(1.0, 0, 0, 0, 'hsr') #relative by omni_base
    ##nav.go_abs(1, 1, 0, 0, 'hsr') #absolute by omni_base
    #nav.go_abs(2.0, 0, 0, 0, 'pumas')#absolute by pumas
    #goal = Pose2D(1.0, 1.3, 1.57)
    #goal = Pose2D(2.0, 3.0, -1.57)
    goal = Pose2D(0.0, 0.0, 0.0)
    #goal = Pose2D(2.30, -0.15, -1.57) #task box
    #goal = Pose2D(2.74, -0.17, -1.57) #unknown box

    #goal = Pose2D(0.14, 0.22, -1.57) #drawer left
    #goal = Pose2D(-0.14, 0.22, -1.57) #drawer left
    #goal = Pose2D(0.95, -0.23, -1.57) #kitchen
    #goal = Pose2D(1.65, -0.20, -1.57) #trayb
    #goal = Pose2D(1.53, -0.20, -1.57) #traya
    #goal = Pose2D(1.15, -0.20, -1.57) #orien
    #goal = Pose2D(-0.54, 0.90, 1.30) #search0


    goal = Pose2D(1.77, 2.0,1.57) 
    nav.nav_goal(goal, nav_type="pumas", nav_mode="abs", nav_timeout=0, goal_distance=0, angle_correction=True, obstacle_detection=False)



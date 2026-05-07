#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

import copy
import math
import numpy as np
from typing import Union, Tuple # go_nav function
from fractions import Fraction

import rospy
import tf

from hsrlib.hsrif import HSRInterfaces 
from hsrlib.rosif import ROSInterfaces

from std_msgs.msg import Float32MultiArray, Bool, Empty, Float32
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Vector3, Quaternion, PoseStamped, Pose2D, Twist, PoseArray
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger, SetBool
from motion_synth.msg import Joints, StartAndEndJoints

#default pose for motion_synth start pose {rad,}
default_arm_pose = {
               'arm_flex_joint': -0.26, #default is 0.0
               'arm_lift_joint': 0.0, 
               'arm_roll_joint': -1.57,
               'wrist_flex_joint': -1.57,
               'wrist_roll_joint': 0.0,
               'head_pan_joint': 0.0,
               'head_tilt_joint': np.deg2rad(0.0),}


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
            rospy.loginfo("NavModule -> called class initializaion")
            return
        self.global_goal_reached = True
        self.goal_reached = True
        self.robot_stop = False

        self.hsrif = HSRInterfaces()
        self.rosif = ROSInterfaces()
        
        # for visualize goal point
        self.marker_num = 0

        self.global_pose = None
        self.marker = Marker()

        self.path_plan_client = rospy.ServiceProxy('/path_planner/path_plan_status', Trigger)
        self.pub_global_goal_xyz = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_move_rel = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_dist_angle = rospy.Publisher('/simple_move/goal_dist_angle', Float32MultiArray, queue_size=1)
        self.pub_robot_stop = rospy.Publisher('/navigation/stop', Empty, queue_size=1)

        # for via_points
        self.via_points = None
        self.pub_via_points = rospy.Publisher('/navigation/via_points', PoseArray, queue_size=1)
        

        self.pub_cmd_vel = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
        self.pub_marker = rospy.Publisher('/nav_goal_marker', Marker, queue_size=10)

        # for motion synth
        self.motion_synth_start_pose = None
        self.motion_synth_end_pose = None
        self.pub_move_joint_pose = rospy.Publisher('/pumas_motion_synth/joint_pose', StartAndEndJoints, queue_size=1)

        # for recovery motion synth is not set
        self.check_default_arm_pose()
        self.arm_pub = rospy.Publisher("/hardware/arm/goal_pose", Float32MultiArray, queue_size=1)
        self.lift_pub = rospy.Publisher("/hardware/torso/goal_pose", Float32, queue_size=1)
        self.head_pub = rospy.Publisher("/hardware/head/goal_pose", Float32MultiArray, queue_size=1)

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
    
    # Load default_arm_pose from ROS param if available 
    def check_default_arm_pose(self):

        #arm lift
        if rospy.has_param('/hardware/arm/torso_goal_pose'):
            torso_goal = rospy.get_param('/hardware/arm/torso_goal_pose')
            try:
                default_arm_pose['arm_lift_joint'] = float(torso_goal)
                rospy.loginfo(f"Loaded torso_goal_pose: {torso_goal}")
            except ValueError:
                rospy.logwarn("torso_goal_pose is not a valid float, using default.")
        else:
            rospy.logwarn("torso_goal_pose parameter not set, using default.")

        #arm&wrist
        if rospy.has_param('/hardware/arm/arm_goal_pose'):
            arm_goal_param = rospy.get_param('/hardware/arm/arm_goal_pose')
            if isinstance(arm_goal_param, list) and len(arm_goal_param) == 4:
                arm_goal_pose = [float(x) for x in arm_goal_param]
                default_arm_pose['arm_flex_joint'] = arm_goal_pose[0]
                default_arm_pose['arm_roll_joint'] = arm_goal_pose[1]
                default_arm_pose['wrist_flex_joint'] = arm_goal_pose[2]
                default_arm_pose['wrist_roll_joint'] = arm_goal_pose[3]
            else:
                rospy.logwarn("arm_goal_pose is not a list, using defaults.")
        else:
            rospy.logwarn("arm_goal_pose parameter not set, using defaults.")

        #head
        default_arm_pose['head_pan_joint'] = 0.0
        default_arm_pose['head_tilt_joint'] = 0.0
        rospy.loginfo(f"Loaded arm_goal_pose: {default_arm_pose}")

    def callback_global_goal_reached(self, msg):

        path_plan_success = self.path_plan_client()
        if path_plan_success.success:
            rospy.loginfo("NavModule -> pathplan success")
        else:
            rospy.loginfo("NavModule -> pathplan fail")

        self.global_goal_reached = False
        if msg.status == GoalStatus.SUCCEEDED:
            self.global_goal_reached = True

        if msg.status == GoalStatus.ACTIVE:
            if msg.text == 'Waiting for temporal obstacles to move':
                rospy.loginfo('NavigationStatus -> Waiting for temporal obstacle to move')
                # TODO rotation
                #self.recovery_from_cost()

        elif msg.status == GoalStatus.ABORTED:
            if msg.text == 'Cannot calculate path from start to goal point':
                rospy.loginfo('NavigationStatus -> Cannot calculate path from start to goal point')
            elif msg.text == 'Cancelling current movement':
                rospy.loginfo('NavigationStatus -> Cancelling current movement')

    def callback_goal_reached(self, msg):
        self.goal_reached = False
        if msg.status == GoalStatus.SUCCEEDED:
            self.goal_reached = True

    def callback_stop(self, msg):
        self.robot_stop = True

    def set_navigation_type(self, type_nav):
        valid_types = ["hsr", "pumas"]
        if type_nav in valid_types:
            self.navigation_setter = type_nav
        else:
            rospy.logerr("NavModule.->Invalid navigation mode")
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

    def pose2d2pose(self, pose2d):
        pose = Pose()
        pose.position.x = pose2d.x
        pose.position.y = pose2d.y
        pose.position.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, pose2d.theta)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

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

    #def recovery_from_cost(self):
    #    # TODO rotate cancel
    #    rospy.loginfo('NavigationStatus. -> Recovery From Dynamic Cost -> Rotation')

    #    current_orientation = self.global_pose.pose.orientation
    #    _, _, current_yaw = tf.transformations.euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])
    #    opposite_yaw = current_yaw + math.pi

    #    twist = Twist()
    #    twist.angular.z = 0.5 if opposite_yaw > current_yaw else - 0.5
    #    rate = rospy.Rate(10)
    #    duration = abs(opposite_yaw - current_yaw) / 0.5

    #    for _ in range(int(duration * 10)):  # duration in seconds, rate is 10 Hz
    #        self.pub_cmd_vel.publish(twist)
    #        rate.sleep()

    #    # Stop rotation
    #    twist.angular.z = 0.0
    #    self.pub_cmd_vel.publish(twist)
    #    rospy.sleep(0.1)

    def get_close(self, x, y, yaw, timeout, goal_distance=None):
        """_summary_

        Args:
            x (_type_): _description_
            y (_type_): _description_
            yaw (_type_): _description_
            timeout (_type_): _description_
            goal_distance (_type_, optional): _description_. Defaults to None.
            via_points (_type_, optional): _description_. Defaults to None.
        """
        rate = rospy.Rate(10)

        via_points_list = []
        if self.via_points is not None:
            rospy.loginfo("enable ViaPoint")
            via_points_array = PoseArray()
            via_points_array.header.frame_id = "map"
            via_points_array.header.stamp = rospy.Time.now()
            for points in self.via_points:
                via_points_list.append(self.pose2d2pose(points))
            via_points_array.poses = via_points_list
            self.pub_via_points.publish(via_points_array)

            rospy.loginfo(f"NavModule. -> via_points: {via_points_list}")

        goal = self.create_goal_pose(x, y, yaw, "map")
        self.global_goal_reached = False
        self.robot_stop = False
        attempts = int(timeout * 10) if timeout != 0 else float('inf')

        self.global_goal_xyz = copy.deepcopy(goal)
        self.send_goal(goal)
        #self.send_goal(goal) TODO with viapoint

        while not self.global_goal_reached and not rospy.is_shutdown() and not self.robot_stop and attempts >= 0:
            if goal_distance:
                now_x, now_y = self.global_pose.pose.position.x, self.global_pose.pose.position.y
                now_distance = math.sqrt((x - now_x) ** 2 + (y - now_y) ** 2)
                if now_distance < goal_distance:
                    break

            attempts -= 1
            rate.sleep()

        self.handle_robot_stop()
        self.via_points = None

    def go_abs(self, x: float, y: float, theta: float, timeout: int=0, type_nav: str=None, goal_distance: float=None, via_points: Tuple[Pose2D]=None):
        """絶対座標ナビゲーション

        Args:
            x (float): ゴールのx座標
            y (float): ゴールのy座標
            theta (float): ゴールのyaw角
            timeout (int, optional): ナビゲーションのタイムアウト. Defaults to 0.
            type_nav (str, optional): ナビゲーションタイプ. Defaults to None.
            goal_distance (float, optional): 許容誤差. Defaults to None.
            via_points (Tuple[Pose2D], optional): via_points. Defaults to None.
        """
        if type_nav == "pumas":
            rospy.loginfo("Call ABS mode in pumas nav")
            if via_points is not None:
                print("with via_points")
                self.via_points = via_points
                self.get_close(x, y, theta, timeout, goal_distance=None)
            else:
                self.via_points = None
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

    def create_arm_joint_goal(self, joint_poses):
        joints = Joints()
        joints.arm_lift_joint = joint_poses["arm_lift_joint"]
        joints.arm_flex_joint = joint_poses["arm_flex_joint"]
        joints.arm_roll_joint = joint_poses["arm_roll_joint"]
        joints.wrist_flex_joint = joint_poses["wrist_flex_joint"]
        joints.wrist_roll_joint = joint_poses["wrist_roll_joint"]
        joints.head_pan_joint = joint_poses["head_pan_joint"]
        joints.head_tilt_joint = joint_poses["head_tilt_joint"]
        return joints

    def send_goal(self, goal):
        rospy.logwarn('NavModule -> Sending Nav Goal')

        # if skip_joint_move is False:
        # TODO bad code
        if self.motion_synth_start_pose is not None or self.motion_synth_end_pose is not None:

            start_and_end_joints = StartAndEndJoints()
            start_and_end_joints.has_arm_start_pose = False
            start_and_end_joints.has_arm_end_pose = False

            if self.motion_synth_start_pose is not None:
                start_and_end_joints.has_arm_start_pose = True
                start_and_end_joints.start_pose = self.create_arm_joint_goal(joint_poses=self.motion_synth_start_pose)
            else:
                start_and_end_joints.has_arm_start_pose = True
                # startが無いなら自動でgo_pose()代入
                start_and_end_joints.start_pose = self.create_arm_joint_goal(joint_poses=default_arm_pose)

            if self.motion_synth_end_pose is not None:
                start_and_end_joints.has_arm_end_pose = True
                start_and_end_joints.end_pose = self.create_arm_joint_goal(joint_poses=self.motion_synth_end_pose)
            else:
                # goalが無い場合
                start_and_end_joints.has_arm_end_pose = False

            self.pub_move_joint_pose.publish(start_and_end_joints)

            rospy.logwarn('NavModule -> Sending Arm Goal')
            start_and_end_joints.has_arm_start_pose = False
            start_and_end_joints.has_arm_end_pose = False

        else:
            #add by r.k 2025/07/04 recovery motion synth is not enable but arm is not default pose
            rospy.logwarn('NavModule -> Sending Recovery Arm Go Pose')

            lift_msg = Float32()
            lift_msg.data = default_arm_pose['arm_lift_joint']
    
            arm_msg = Float32MultiArray()
            arm_msg.data = [
                default_arm_pose['arm_flex_joint'],
                default_arm_pose['arm_roll_joint'],
                default_arm_pose['wrist_flex_joint'],
                default_arm_pose['wrist_roll_joint'],
            ]
    
            head_msg = Float32MultiArray()
            head_msg.data = [
                default_arm_pose['head_pan_joint'],
                default_arm_pose['head_tilt_joint'],
            ]
    
            self.lift_pub.publish(lift_msg)
            self.arm_pub.publish(arm_msg)
            self.head_pub.publish(head_msg)

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
        rospy.sleep(0.1)

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
        rospy.sleep(0.1)

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


    def handle_robot_stop(self):
        if not self.global_goal_reached:
            msg_stop = Empty()
            self.pub_robot_stop.publish(msg_stop)
            rospy.sleep(0.1)
        rospy.sleep(0.1)

    def rotate_yaw(self, goal_pose):

        pi = 3.1415926535
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
        rospy.set_param('/obs_detector/use_point_cloud', status)
        rospy.loginfo(f"NavModule.-> obstacle_detection use POINT CLOUD >>  {status}")
        #rospy.set_param('/obs_detector/use_lidar', status)
        #rospy.logwarn(f"NavModule.-> obstacle_detection use LIDAR >>  {status}")

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
    def nav_goal(self, goal: Union[Pose2D, str], motion_synth_pose=None, nav_type = "pumas", nav_mode = "abs", nav_timeout = 0, goal_distance = 0.0, angle_correction=False, obstacle_detection=True, via_points=None):
         """ _NavModulePumas_
         Args:
         goal (Pose2D): Final Position given by x,y,yaw
         ms_config (Dict): {"start_pose": start_pose, "goal_pose": goal_pose}
         nav_type(pumas_nav) (str): pumas_nav -> "pumas"(default) , toyota_nav -> "hsr"
         nav_mode(pumas_nav) (str): "abs" or "rel", default -> abs
         nav_timeout(pumas_nav) (Float):50.0 -> 50s, 0 -> infinity
         goal_distance(pumas_nav, only abs mode) (Float): goal position - goal_distance
         """
         rospy.loginfo(goal)

         # under constructoin: obstacle_detection (point_cloud) can set to only on/off. 
         if obstacle_detection:
             self.use_obstacle_detection(status=True)
         else:
             self.use_obstacle_detection(status=False)

         #motion_synth
         if motion_synth_pose is not None:
            start_pose = motion_synth_pose.get("start_pose")
            end_pose = motion_synth_pose.get("goal_pose")

            #head cloud is off
            self.use_obstacle_detection(status=False)

            if start_pose:
                rospy.loginfo(f"NavModule. -> Enable MotionSynth for PumasNav. Start Pose -> {start_pose}")
                self.motion_synth_start_pose = start_pose
            else:
                rospy.loginfo(f"NavModule. -> Enable MotionSynth for PumasNav. Start Pose is not set. Calling default Pose -> {start_pose}")
                self.motion_synth_start_pose = start_pose
            if end_pose:
                rospy.loginfo(f"NavModule. -> Enable MotionSynth for PumasNav. End Pose -> {end_pose}")
                self.motion_synth_end_pose = end_pose


         self.via_points = list(via_points) if via_points else None

         if nav_mode == "rel":
             self.go_rel(goal.x, goal.y, goal.theta, nav_timeout, nav_type)
         else:
             self.go_abs(goal.x, goal.y, goal.theta, nav_timeout, nav_type, goal_distance, via_points=self.via_points)

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
    #goal = Pose2D(2.30, -0.15, -1.57) #task box
    #goal = Pose2D(2.74, -0.17, -1.57) #unknown box

    #goal = Pose2D(0.14, 0.22, -1.57) #drawer left
    #goal = Pose2D(-0.14, 0.22, -1.57) #drawer left
    #goal = Pose2D(0.95, -0.23, -1.57) #kitchen
    #goal = Pose2D(1.65, -0.20, -1.57) #trayb
    #goal = Pose2D(1.53, -0.20, -1.57) #traya #goal = Pose2D(1.15, -0.20, -1.57) #orien
    #goal = Pose2D(-0.54, 0.90, 1.30) #search0


    #goal = Pose2D(0.0, 0.0, 0.0)
    #goal = Pose2D(2.0, -1.5, 1.57) 
    #nav.nav_goal(goal, nav_type="pumas", nav_mode="abs", nav_timeout=0, goal_distance=0, angle_correction=True, obstacle_detection=False)
    #nav.nav_goal(goal, nav_type="hsr", nav_mode="abs", nav_timeout=0, goal_distance=0, angle_correction=True, obstacle_detection=False)

    #while True:

    #goal = Pose2D(0.8, 1.32, 0.0)
    #goal = Pose2D(.0, .0, 0.0)
    #goal = Pose2D(0.5, 3.8, 0.0)
    #goal = Pose2D(3.0, 0.8, 0.0)
    #goal = Pose2D(5.3, 4.4, 0.0)

    goal = Pose2D(6.0, 0.0, 0.0)
    goal = Pose2D(2.0, 0.0, 0.0)
    goal = Pose2D(8.0, 3.0, 0.0)

    goal = Pose2D(5.6, -2.8, 0.0)
    goal = Pose2D(1.2, 0.0, 0.0)
    
    start_pose = {
        "arm_lift_joint": 0.0,
        "arm_flex_joint": np.deg2rad(0.0),
        "arm_roll_joint": np.deg2rad(0.0),
        "wrist_flex_joint": np.deg2rad(-90.0),
        "wrist_roll_joint": 0.0,
        "head_pan_joint": 0.0,
        "head_tilt_joint": np.deg2rad(0.0),
    }
    goal_pose = {
        "arm_lift_joint": 0.4,
        "arm_flex_joint": np.deg2rad(-90.0),
        "arm_roll_joint": np.deg2rad(0.0),
        "wrist_flex_joint": np.deg2rad(-90.0),
        "wrist_roll_joint": 0.0,
        "head_pan_joint": 0.0,
        "head_tilt_joint": np.deg2rad(0.0),
    }
    
    #ms_config = {"start_pose": "auto", "goal_pose": goal_pose}
    ms_config = {"start_pose": start_pose}
    ms_config = None
    ms_config = {"goal_pose": goal_pose}
    ms_config = {"start_pose": start_pose, "goal_pose": goal_pose}
    
    #nav.nav_goal(goal, nav_type="hsr", nav_mode="rel", nav_timeout=0, goal_distance=0, angle_correction=False, obstacle_detection=False)
    #nav.nav_goal(goal, motion_synth_start_pose=arm_start_pose, motion_synth_end_pose=arm_end_pose, nav_type="pumas", nav_mode="abs", nav_timeout=0, goal_distance=0, angle_correction=False, obstacle_detection=False)
    #nav.nav_goal(goal, motion_synth_start_pose=arm_start_pose, motion_synth_end_pose=arm_end_pose, nav_type="pumas", nav_mode="abs", nav_timeout=0, goal_distance=0, angle_correction=False)

    #nav.nav_goal(goal, motion_synth_pose=ms_config, nav_type="pumas", nav_mode="abs", nav_timeout=0, goal_distance=0, angle_correction=False)
    #goal = Pose2D(2.31, 5.6, 0.0)

    via_points = [Pose2D(2.3, 0.0, 0.0),  Pose2D(2.0, 4.2, 0.0)]
    #via_points = None
    #nav.nav_goal(goal, motion_synth_pose=ms_config, nav_type="pumas", nav_mode="abs", nav_timeout=0, goal_distance=0, angle_correction=False, via_points=via_points)
    nav.nav_goal(goal, motion_synth_pose=ms_config, nav_type="pumas", nav_mode="abs", nav_timeout=0, goal_distance=0, angle_correction=False, via_points=None)
    

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
#import hsrb_interface
from hsrlib.hsrif import HSRInterfaces

import tf

from std_msgs.msg import Float32MultiArray, Bool, Empty
from nav_msgs.msg import Path

from geometry_msgs.msg import Pose, Vector3, Quaternion, PoseStamped, Pose2D, Twist
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker


from std_srvs.srv import Trigger
import copy

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

        #self.robot = hsrb_interface.Robot()
        #self.whole_body.omni_base = self.robot.get("omni_base")
        #self.whole_body = self.robot.get("whole_body")
        self.hsrif = HSRInterfaces()
        #self.whole_body.omni_base = self.hsrif.omni_base
        #self.whole_body = self.hsrif.whole_body
        
        self.distance_from_cost = 0.1
        self.marker_num = 0
        self.points = []

        self.set_navigation_type(select)
        #self.global_pose = PoseStamped()
        #self.global_goal_xyz = PoseStamped()
        # self.global_goal_xyz = None
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

        self.is_inside_obstacles = rospy.ServiceProxy('/map_augmenter/is_inside_obstacles', Trigger)
        self.are_there_obstacles = rospy.ServiceProxy('/map_augmenter/are_there_obstacles', Trigger)

        rospy.sleep(1.0)

    def callback_global_goal_reached(self, msg):

        #rospy.logwarn('#######before service#########')

        #rospy.logerr(self.points)

        path_plan_success = self.path_plan_client()
        if path_plan_success.success:
            rospy.loginfo("NavModule -> pathplan success")
            self.points = []
            self.distance_from_cost = 0.1
            #rospy.logerr(self.points)
        else:
            rospy.loginfo("NavModule -> pathplan fail")

        #rospy.logwarn('#######after service#########')
        #print("callback navigation self.global_goal_xyz:", self.global_goal_xyz)
        self.global_goal_reached = False
        if msg.status == GoalStatus.SUCCEEDED:
            self.global_goal_reached = True

        if msg.status == GoalStatus.ACTIVE:
            if msg.text == 'Waiting for temporal obstacles to move':
                rospy.loginfo('NavigationStatus -> Waiting for temporal obstacle to move')

                #if len(self.points) == 0:
                #self.recovery_from_cost()
                if not self.points:
                   self.replan_safe_point()
                   self.distance_from_cost += 0.2
                else:
                   self.publish_next_point()

        elif msg.status == GoalStatus.ABORTED:
            if msg.text == 'Cannot calculate path from start to goal point':
                rospy.loginfo('NavigationStatus -> Cannot calculate path from start to goal point')

                if not self.points:
                    self.replan_safe_point()
                    self.distance_from_cost += 0.2
                else:
                    self.publish_next_point()

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
            #rospy.logerr(f'points: {self.points}')

            #if not len(self.points) == 0:
            if self.points:
                self.publish_next_point()
            else:
                self.hsrif.tts.say('Path Planning failed. I will try again.', language='en', queue=True, sync=True)
                rospy.loginfo('called replan_safe_point -> but not found')
                #self.distance_from_cost += 0.2

        except:
            import traceback
            traceback.print_exc()

            

    def publish_next_point(self):
        rospy.loginfo("called publish next point")
        #try:
        #if not len(self.points) == 0:
        #if self.points:
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

        # print('remove 1st points: ', goal)
        self.send_goal(goal)
        #self.marker_plot(goal)
        #self.pub_global_goal_xyz.publish(goal)
    
        #except Exception as e:
        #    print(e)

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
        self.hsrif.tts.say('Obstacle detected.', language='en')

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
        rospy.sleep(5.0)

        while not self.global_goal_reached and not rospy.is_shutdown() and not self.robot_stop and attempts >= 0:
            if goal_distance:
                now_x, now_y = self.global_pose.pose.position.x, self.global_pose.pose.position.y
                now_distance = math.sqrt((x - now_x) ** 2 + (y - now_y) ** 2)
                if now_distance < goal_distance:
                    break

            attempts -= 1
            rate.sleep()

        self.handle_robot_stop()

    def go_abs(self, x, y, theta, timeout=0.0, type_nav=None, goal_distance=None):
        if type_nav == "pumas":

            self.get_close(x, y, theta, timeout, goal_distance)
        elif type_nav == "hsr":
            self.whole_body.omni_base.go_abs(x, y, theta, timeout)
        else:
            if self.navigation_setter == "pumas":
                self.get_close(x, y, theta, timeout)
            else:
                self.whole_body.omni_base.go_abs(x, y, theta, timeout)

    

    def send_goal(self, goal):
        rospy.logwarn('NavModule -> sending new goal')
        #rospy.logwarn(goal)
        self.marker_plot(goal)
        self.pub_global_goal_xyz.publish(goal)


    def move_dist_angle(self, x, yaw, timeout):
        rate = rospy.Rate(10)
        goal = Float32MultiArray()
        goal.data = [x, yaw]

        self.goal_reached = False
        self.robot_stop = False
        attempts = int(timeout * 10) if timeout != 0 else float('inf')

        self.pub_dist_angle.publish(goal)
        rospy.sleep(5.0)

        while not self.goal_reached and not rospy.is_shutdown() and not self.robot_stop and attempts >= 0:
            attempts -= 1
            rate.sleep()

        self.handle_robot_stop()

    def go_dist_angle(self, x=0.0, yaw=0.0, timeout=0.0, type_nav=None):
        self.move_dist_angle(x, yaw, timeout)

    def move_rel(self, x, y, yaw, timeout):
        rate = rospy.Rate(10)
        goal = self.create_goal_pose(x, y, yaw, "base_link")

        self.global_goal_reached = False
        self.robot_stop = False
        attempts = int(timeout * 10) if timeout != 0 else float('inf')
    
        self.pub_move_rel.publish(goal)
        rospy.sleep(5.0)

        while not self.global_goal_reached and not rospy.is_shutdown() and not self.robot_stop and attempts >= 0:
            attempts -= 1
            rate.sleep()

        self.handle_robot_stop()

    def go_rel(self, x=0.0, y=0.0, yaw=0.0, timeout=0.0, type_nav=None):
        rospy.loginfo("Call REL mode in pumas nav")
        if type_nav == "pumas":
            self.move_rel(x, y, yaw, timeout)
        elif type_nav == "hsr":
            self.hsrif.omni_base.go_rel(x, y, yaw, timeout)
        else:
            if self.navigation_setter == "pumas":
                self.move_rel(x, y, yaw, timeout)
            else:
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
            rospy.sleep(5.0)
        rospy.sleep(2.5)

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
        return self.hsrif.omni_base.get_pose

if __name__ == "__main__":
    rospy.init_node('navigation_module')
    nav = NavModule(select="hsr")

    # example usage
    nav.go_abs(6.45, -1.8, -1.57, 0, 'pumas')
    #nav.go_abs(3.68, -1.65, -1.57, 0, 'pumas')
    #nav.go_abs(2.14, 1.5, -1.57, 0, 'pumas')
    #nav.go_abs(0, 0, -1.57, 0, 'pumas')

    #i = 0
    #while not rospy.is_shutdown():
    #    nav.go_abs(0, 0, 0, 0, 'pumas')
    #    rospy.sleep(0.5)
    #    i += 1
    #    print('num_try: move2originPoint ',i)


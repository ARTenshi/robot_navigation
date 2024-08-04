#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#

import rospy
import smach
import numpy as np
import math

from hsrlib.hsrif import HSRInterfaces
from hsrlib.utils import utils, description, joints
from tamlib.utils import Logger

from geometry_msgs.msg import Pose2D, PointStamped


from navigation_tools.nav_tool_lib import NavModule
#from common.speech import DefaultTTS

from std_msgs.msg import Bool
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import LaserScan

from human_position.msg import HumanCoordinatesArray, Keypoint

#hsrb 71,hsrc55
#THRESHOLD = 12.0
#hsrb 22
#THRESHOLD = 21.0


class FollowPerson(smach.State, Logger):
    def __init__(self, move_joints = True, timeout=None):
        smach.State.__init__(self, outcomes=['next', 'except'], input_keys=['way_point_list'], output_keys=['way_point_list'])
        Logger.__init__(self, loglevel="DEBUG")

        self.hsrif = HSRInterfaces()

        self.nav = NavModule()
        
        #self.robot = robot
        #self.whole_body = self.robot.get('whole_body')
        #self.omni_base = self.robot.get("omni_base")
        #self.gripper = self.robot.get('gripper')

        #self._tts = DefaultTTS()


        self.fp_enable_leg_finder_pub = rospy.Publisher(
            '/hri/leg_finder/enable', Bool)
        self.fp_start_follow_pub = rospy.Publisher(
            '/hri/human_following/enable', Bool)

        self.fp_legs_found_sub = rospy.Subscriber(
            '/hri/leg_finder/legs_found', Bool, self._fp_legs_found_cb)

        self.leg_pose_sub = rospy.Subscriber(
            '/hri/leg_finder/leg_pose', PointStamped, self.legs_pose_cb)

        self.fp_legs_found = False
        self.fisrt = True
        self.distance_flag = False

        #false -> go pose , true -> custom_pose
        self.move_joints = move_joints

        #set custom_pose
        self.follow_pose = joints.get_go_pose()
        self.follow_pose["wrist_flex_joint"] = np.deg2rad(-90.0)
        self.follow_pose["wrist_roll_joint"] = np.deg2rad(0.0)
        self.follow_pose["arm_lift_joint"]   = 0.26
        self.follow_pose["arm_roll_joint"]   = 2.0
        self.follow_pose["arm_flex_joint"]   = -0.16

        self.closest_distance = False
        self.min_distance = False

        # for update pot_fields
        #self.pot_fields_d0 = rospy.get_param('obs_detector/pot_fields_d0')
        #self.pot_fields_k_rej = rospy.get_param('obs_detector/pot_fields_k_rej')
        #self.update_potfields = rospy.Publisher('/navigation/pot_fields_update', Bool, queue_size=10)
        #self.update = Bool()
        #self.update.data = False
        #self.update_potfields.publish(self.update.data)

        #self.obs_detect_enable = rospy.Publisher('/navigation/obs_detector/enable', Bool, queue_size=10)

    def legs_pose_cb(self, msg): 
        self.leg_pose = msg
              
        if self.leg_pose.point.x > 1.5:
            if not self.distance_flag:                                                          
                self.hsrif.tts.say('Sorry, Please walk more slowly.', language='en', sync=True, queue=True) # moreをつけました
                rospy.sleep(5)
                self.hsrif.tts.say('If you want me to stop following you, touch my hand.', language='en', sync=True, queue=True)
                self.distance_flag = True  # Set flag to True to indicate message has been spoken
        else:
            self.distance_flag = False  # Reset flag when person is within acceptable distance

        #if self.leg_pose.point.x > 1.5:
        #    self.hsrif.tts.say('Sorry, Please walk slowly.', language='en', sync=True, queue=True)
        #    self.distance_flag == True



    def _fp_legs_found_cb(self, msg):
        try:
            if msg.data == True and msg.data != self.fp_legs_found:
                self.fp_legs_found = True

                self.hsrif.tts.say('I found you! Now, I will follow you.', language='en', sync=True, queue=True)
                
                if (self.fisrt):
                    self.hsrif.tts.say('If you want me to stop following you, touch my hand.', language='en', sync=True, queue=True)
                    self.fisrt = False

                
                rospy.loginfo('Legs found')


            elif msg.data == False and msg.data != self.fp_legs_found:
                self.fp_legs_found = False
                self.distance_flag == False

                self.hsrif.tts.say(
                    'Sorry, I lost you! Please come where I can see you.', language='en', sync=True, queue=True)
                rospy.loginfo('Legs lost')
        except:
            self.distance_flag == False
            self.fp_legs_found = False

    def distance2center_hip(self, data):

        people = data.human_coordinates_array

        closest_distance = float('inf')

        if people is None:
            return
        try:
            for person in people:
                keypoint = {kpt.name:kpt for kpt in person.keypoints if kpt.name in ["CENTER_HIP"]}

                if "CENTER_HIP" in keypoint:
                    self.loginfo("Success to get hip_point.")

                    center_hip = keypoint["CENTER_HIP"]
                    distance = math.sqrt(center_hip.point.z**2)

                    if distance < closest_distance:
                        closest_distance = distance
                        
            self.closest_distance = closest_distance

        except Exception as e:
            self.logdebug(f"{e}")
            pass
    
    def distance_leg(self, data):
        ranges = data.ranges
        angle_increment = data.angle_increment
        min_angle = data.angle_min

        min_detect_angle = -15.0 * 3.14/180.0
        max_detect_angle = 15.0 * 3.14/180.0

        detected_distances = []
        detected_angles = []

        for i, distance in enumerate(ranges):
            angle = min_angle + i * angle_increment
            if min_detect_angle <= angle <= max_detect_angle:
                detected_distances.append(distance)
                detected_angles.append(angle)

        if detected_distances:
            min_distance = min(detected_distances)

        self.min_distance = min_distance
        print(self.min_distance)

    def execute(self, userdata):
        try:
            rate = rospy.Rate(30)
            rospy.loginfo("exucute")
            
            #if self.move_joints:
            self.hsrif.whole_body.move_to_joint_positions(self.follow_pose)
            #self.obs_detect_enable = rospy.Publisher('/navigation/obs_detector/enable', Bool, queue_size=10)
            #else:
            #self.hsrif.whole_body.move_to_go()


            self.fp_enable_leg_finder_pub.publish(False)
            self.fp_start_follow_pub.publish(False)

            rospy.loginfo("first pose")

            self.hsrif.tts.say('Operator, please come in front of me.', language='en', sync=True)
            rospy.sleep(1)

            # 足の位置と腰の位置の確認
            hip_sub = rospy.Subscriber('/human_coordinates', HumanCoordinatesArray, self.distance2center_hip)
            leg_sub = rospy.Subscriber("/hsrb/base_scan", LaserScan, self.distance_leg)

            start_time = rospy.get_time()

            while not rospy.is_shutdown():
                cur_time = rospy.get_time()

                if cur_time - start_time <= 10:
                    if self.closest_distance and self.min_distance:
                        if self.min_distance < 1.0 and (self.closest_distance - self.min_distance) < 0.4:
                            hip_sub.unregister()
                            leg_sub.unregister()
                            break
                        else:
                            rate.sleep()
                            continue
                    else:
                        rate.sleep()
                        continue
                else:
                    hip_sub.unregister()
                    leg_sub.unregister()
                    break

            self.hsrif.tts.say("Touch my hand to start following you.", 'en', sync=True)

            rospy.loginfo("Touch my hand to start following you.")

            #self.update.data = True
            #self.update_potfields.publish(self.update.data)
            #self.obs_detect_enable.publish(self.update.data)

            while not utils.is_arm_touched():

                rate.sleep()

            self.hsrif.tts.say('First I will find you. Please, move in front of me, where I can see you.', language='en', sync=True, queue=True)

            rospy.loginfo("First I will find you.")
            self.fp_enable_leg_finder_pub.publish(True)


            way_point_list = []
            last_pose_time = rospy.get_time()
            say_counter = 0
            while not utils.is_arm_touched():

                if self.fp_legs_found == False:
                    self.fp_start_follow_pub.publish(False)
                    self.distance_flag = False


                    while self.fp_legs_found == False:

                        rate.sleep()

                    self.fp_start_follow_pub.publish(True)
                #rate.sleep()


                current_time = rospy.get_time()
                #rospy.loginfo("current_time")
                diff =  current_time - last_pose_time

                if diff >= 5:
                    nav_pose = self.nav.pose()
                    current_pose = Pose2D(nav_pose().x, nav_pose().y, nav_pose().theta)
                    
                    if current_pose:
                        way_point_list.append(current_pose)

                    last_pose_time = current_time

                rate.sleep()

            userdata.way_point_list = way_point_list
            rospy.logwarn(userdata.way_point_list)

            self.fp_legs_found = False
            self.distance_flag = False

            self.fp_enable_leg_finder_pub.publish(False)
            self.fp_start_follow_pub.publish(False)

            #self.hsrif.whole_body.move_to_go()

            
            self.hsrif.tts.say('OK, I will stop following you.', language='en', sync=True, queue=True)
            rospy.loginfo('OK, I will stop following you.')

            #self.update.data = False
            #self.update_potfields.publish(self.update.data)

            return 'next'

            # return 'timeout'
        except:
            import traceback
            traceback.print_exc()
            self.fp_legs_found = False
            self.distance_flag = False

            self.fp_enable_leg_finder_pub.publish(False)
            self.fp_start_follow_pub.publish(False)
            #self.update.data = False
            #self.update_potfields.publish(self.update.data)

            return 'except'

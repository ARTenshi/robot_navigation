#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

import rospy
import smach

from common.speech import DefaultTTS

from std_msgs.msg import Bool
from geometry_msgs.msg import WrenchStamped

THRESHOLD = 12.0


class FollowPerson(smach.State):
    def __init__(self, robot, move_go = True, timeout=None):
        smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'])

        self.robot = robot
        self.whole_body = self.robot.get('whole_body')
        self.omni_base = self.robot.get("omni_base")
        self.gripper = self.robot.get('gripper')

        self._tts = DefaultTTS()

        self.fp_enable_leg_finder_pub = rospy.Publisher(
            '/hri/leg_finder/enable', Bool)
        self.fp_start_follow_pub = rospy.Publisher(
            '/hri/human_following/enable', Bool)

        self.fp_legs_found_sub = rospy.Subscriber(
            '/hri/leg_finder/legs_found', Bool, self._fp_legs_found_cb)
        self.fp_wrist_wrench_sub = rospy.Subscriber(
            '/hsrb/wrist_wrench/raw', WrenchStamped, self._wrist_wrench_cb)

        self.move_go = move_go

        self.fp_legs_found = False
        self.fisrt = True

        self.pushed = False

    def _fp_legs_found_cb(self, msg):
        try:
            if msg.data == True and msg.data != self.fp_legs_found:
                self.fp_legs_found = True

                self._tts.say('I found you! Now, I will follow you.')
                
                if (self.fisrt):
                    self._tts.say('If you want me to stop following you, push my hand.')
                    self.fisrt = False
                
                rospy.loginfo('Legs found')

            elif msg.data == False and msg.data != self.fp_legs_found:
                self.fp_legs_found = False

                self._tts.say(
                    'Sorry, I lost you! Please come where I can see you.')
                rospy.loginfo('Legs lost')
        except:
            self.fp_legs_found = False

    def _wrist_wrench_cb(self, msg):
        try:
            self.pushed = False
            current_value = msg.wrench.force.x
            if THRESHOLD > 0.:
                if current_value > THRESHOLD:
                    self.pushed = True
            else:
                if current_value < -THRESHOLD:
                    self.pushed = True
        except:
            self.pushed = False

    def execute(self, userdata):
        try:
            rate = rospy.Rate(30)
            
            if self.move_go:
                self.whole_body.move_to_go()

            self.fp_enable_leg_finder_pub.publish(False)
            self.fp_start_follow_pub.publish(False)

            self._tts.say('Push my hand to start following you.')

            while self.pushed == False:
                rate.sleep()

            self._tts.say('First I will find you. Please, move in front of me, where I can see you.')
            self.fp_enable_leg_finder_pub.publish(True)

            while self.pushed == False:
                if self.fp_legs_found == False:
                    self.fp_start_follow_pub.publish(False)
                    while self.fp_legs_found == False:
                        rate.sleep()

                    self.fp_start_follow_pub.publish(True)

                rate.sleep()

            self.fp_legs_found = False

            self.fp_enable_leg_finder_pub.publish(False)
            self.fp_start_follow_pub.publish(False)

            self.whole_body.move_to_go()

            self._tts.say('OK, I will stop following you.')

            return 'success'

            # return 'timeout'
        except:
            self.fp_legs_found = False

            self.fp_enable_leg_finder_pub.publish(False)
            self.fp_start_follow_pub.publish(False)

            return 'failure'

#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ROS library
import rospy
import smach
import traceback
import logging

# HSRB Library
import hsrb_interface

from tf.transformations import quaternion_matrix, quaternion_from_euler, quaternion_from_matrix

from common import speech
from common.follow_person import FollowPerson
from common.wait_hand_pushed import WaitHandPushed

from navigation_tools.nav_tool_lib import nav_module

##################################################

# Main
robot = hsrb_interface.Robot()

whole_body = robot.get("whole_body")
#omni_base = robot.get("omni_base") #Standard initialisation (Toyota)
omni_base = nav_module("hsr") #New initalisation (Pumas)
gripper = robot.get('gripper')
tf_buffer = robot._get_tf2_buffer()

default_tts = speech.DefaultTTS()
console = speech.Console()
SAY = default_tts.say

whole_body.move_to_neutral()

rospy.loginfo('initializing...')
rospy.sleep(3)

def create_sm():

  sm = smach.StateMachine(outcomes=['success', 'failure', 'next', 'except'])

  with sm:

        ##########
        # START: TASK INITIALISATION
        ##########
        # Ask for the hand to be pushed to start
        smach.StateMachine.add('WAIT_HAND', WaitHandPushed(timeout=120.,
                                                           say_fn=SAY,
                                                           prompt_msg="Push the hand to start",
                                                           success_msg=""),
                               transitions={'success': 'FOLLOWPERSON',
                                            'timeout': 'FOLLOWPERSON',
                                            'failure': 'failure'})
        ##########
        # END: TASK INITIALISATION
        ##########

        ##########
        #START: FOLLOW PERSON
        ##########
        smach.StateMachine.add('FOLLOWPERSON', FollowPerson(robot),
                               transitions = {'next': 'MOVE2STANDBY',
                                              'except': 'failure'})
        ##########
        #END: FOLLOW PERSON
        ##########

        ##########
        # START: BASIC MOVE SEQUENCE
        ##########
        # Moves the robot to the neutral position
        @smach.cb_interface(outcomes=['success', 'failure'])
        def move_to_standby_cb(userdata):
            try:
                whole_body.move_to_neutral()

                return 'success'
            except:
                return 'failure'

        smach.StateMachine.add('MOVE2STANDBY', smach.CBState(move_to_standby_cb),
                               transitions={'success': 'success',
                                            'failure': 'failure'})

        # Moves the robot to the neutral position
        @smach.cb_interface(outcomes=['success', 'failure'])
        def move_to_timeout_cb(userdata):
            try:
                whole_body.move_to_neutral()
                SAY('The time is over.')
                rospy.sleep(2)

                return 'success'
            except:
                return 'failure'

        smach.StateMachine.add('MOVE2TIMEOUT', smach.CBState(move_to_timeout_cb),
                               transitions={'success': 'success',
                                            'failure': 'failure'})
        ##########
        # END: BASIC MOVE SEQUENCE
        ##########

  return sm


if __name__ == '__main__':
    
    sm = create_sm()
    outcome = sm.execute()
    
    if outcome == 'success':
        SAY('I finished the task.')
        rospy.loginfo('I finished the task.')
    else:
        SAY('Sorry, something happend. I did not finish the task.')
        rospy.signal_shutdown('Some error occured.')
    
    logging.disable(logging.CRITICAL)

#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ROS library
import rospy
import smach
import traceback
import logging
import os

# HSRB Library
import hsrb_interface

from common import speech
from common.follow_person import FollowPerson
from common.wait_hand_pushed import WaitHandPushed

from navigation_tools.nav_tool_lib import nav_module
    
##################################################

# Main
robot = hsrb_interface.Robot()
whole_body = robot.get("whole_body")
#omni_base = robot.get("omni_base") # Standard initialisation (Toyota)
omni_base = nav_module("hsr") # New initalisation (Pumas)
gripper = robot.get('gripper')

###
default_tts = speech.DefaultTTS()
console = speech.Console()
SAY = default_tts.say

# INITIALIZATION
os.system("rosnode kill viewpoint_controller")

whole_body.move_to_neutral()
rospy.loginfo('initializing...')
rospy.sleep(3)

def create_sm():
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    #### 
        
    with sm:
        ##########
        # START: TASK INITIALISATION
        ##########
        # Wait for the task to start
        smach.StateMachine.add('WAIT_HAND', WaitHandPushed(timeout=120.,
                                                           #threshold=-15.0,
                                                           say_fn=SAY,
                                                           prompt_msg="Push the hand to start",
                                                           success_msg="I will start the carry my luggage task."),
                               transitions={'success': 'TAKEBAG',
                                            'timeout': 'WAIT_HAND',
                                            'failure': 'failure'})
        ##########
        # END: TASK INITIALISATION
        ##########                                    

        @smach.cb_interface(outcomes=['success', 'failure'])
        def take_bag_cb(userdata):
            try:
                whole_body.move_to_neutral()
                gripper.command(0.9)
                
                SAY("Please put the bag in my hand from the middle to prevent blocking my base laser. I will close my hand in 3.",wait_result=True)
                rospy.sleep(1)
                SAY('2',wait_result=True)
                rospy.sleep(1)
                SAY('1',wait_result=True)
                rospy.sleep(1)
                
                gripper.apply_force(1.0)
                rospy.sleep(2)
                                
                whole_body.move_to_go()
                
                return 'success'
            except:
                traceback.print_exc()
                return 'failure'

        smach.StateMachine.add('TAKEBAG', smach.CBState(take_bag_cb),
                               transitions={'success': 'FOLLOWPERSON', 
                                            'failure': 'FOLLOWPERSON'}) 
        ##########
        # END: POINT TO ONE'S BAG (Pick and take a guest's bag from the floor.)
        ##########                                              
           

        ##########
        # START: FOLLOW PERSON
        ##########     
        smach.StateMachine.add('FOLLOWPERSON', FollowPerson(robot, move_go = False),
                               transitions = {'success': 'OPENHAND',
                                              'failure': 'OPENHAND',
                                              'timeout': 'OPENHAND'})
                                              
        @smach.cb_interface(outcomes=['success', 'failure'])
        def open_hand_cb(userdata):
            try:
                whole_body.move_to_neutral()
                
                SAY("Please receive this bag in 3.",wait_result=True)
                rospy.sleep(1)
                SAY('2',wait_result=True)
                rospy.sleep(1)
                SAY('1',wait_result=True)
                rospy.sleep(1)
                
                gripper.command(1.2)
                rospy.sleep(2)
                
                return 'success'
            except:
                traceback.print_exc()
                return 'failure'

        smach.StateMachine.add('OPENHAND', smach.CBState(open_hand_cb),
                               transitions={'success': 'MOVE2LOCATION', 
                                            'failure': 'failure'})                                                                    
        ##########
        # END: FOLLOW PERSON
        ##########                                              

        ##########
        # START: RETURN TO A LOCATION
        ##########

        @smach.cb_interface(outcomes=['success', 'failure'])
        def move_to_location_cb(userdata):
            try:
                SAY("I will go back to areana", wait_result=True)
                whole_body.move_to_go()
                
                #Return inside the a location while avoiding obstacles
                omni_base.go_abs(0.0, 0.0, 0.0, 200, "pumas")
                
                return 'success'
              
            except:
                traceback.print_exc()
                return 'failure'

        smach.StateMachine.add('MOVE2LOCATION', smach.CBState(move_to_location_cb),
                               transitions={'success': 'success',
                                            'failure': 'MOVE2LOCATION'})
        ##########
        # END: RETURN TO A LOCATION
        ##########

    return sm

if __name__ == '__main__':
    
    sm = create_sm()
    outcome = sm.execute()
    
    if outcome == 'success':
        rospy.loginfo('I finished the task.')
        SAY("I finished the task")
    else:
        rospy.signal_shutdown('Some error occured.')
    
    logging.disable(logging.CRITICAL)

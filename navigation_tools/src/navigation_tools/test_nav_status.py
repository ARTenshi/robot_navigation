#!/usr/bin/env python3

import rospy
from actionlib_msgs.msg import GoalStatus

#get_most_safe_point
from navigation_tools.search_safe_point import SearchSafePoint

from geometry_msgs.msg import Pose2D

class CheckNavStatus():
    def __init__(self):
        rospy.init_node('check_navigation_status')

        self.search_point = SearchSafePoint()

        self.sub = rospy.Subscriber('/navigation/status', GoalStatus, self.callback)

        self.goal = Pose2D( x = 4.65
                           ,y = -0.36
                           ,theta = 0)


    def callback(self, msg):
        status = msg.status
        text   = msg.text
        rospy.loginfo(msg)


        ###GoalStatus:Active###
        if status == 1:

            #if text == 'Near goal point':
            #    rospy.loginfo('Near Goal')
            #    pass

            if text == 'Waiting for temporal obstacles to move':
                rospy.loginfo('NavigationStatus -> Waiting for temporal obstacle to move')
                #TODO timeout

                safe_point = self.search_point.get_most_safe_point(self.goal , goal_torelance=5, refer_size=5)
                rospy.logerr(safe_point)
                
        ###SUCCEEDED###
        if status == 3:
            #Global goal point reached
            rospy.loginfo('NavigationStatus -> Goal Success')
            return 'success'

        ###ABORTED###
        if status == 4:

            if text == 'Cannot calculate path from start to goal point':
                rospy.loginfo('NavigationStatus -> Cannot calculate path from start to goal point') 

            if text == 'Cancelling current movement':
                rospy.loginfo('Cancelling current movement')
                return ''


if __name__ == '__main__':
    CheckNavStatus()
    rospy.spin()


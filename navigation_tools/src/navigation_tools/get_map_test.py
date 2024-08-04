#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.srv import GetMap, GetMapRequest, GetMapResponse

def call_augment_map_service():
    rospy.init_node('map_augmenter_client')

    rospy.wait_for_service('/map_augmenter/get_augmented_map')
    try:
        augment_map = rospy.ServiceProxy('/map_augmenter/get_augmented_map', GetMap)
        
        request = GetMapRequest()
        
        response = augment_map(request)
        
        #if response:
        #    rospy.loginfo("Successfully received the augmented map.")
        #    return response.map
        #else:
        #    rospy.logwarn("Service call failed.")
        #    return None
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None

if __name__ == "__main__":
    augmented_map = call_augment_map_service()
    if augmented_map:
        rospy.loginfo("Augmented map received and ready for further processing.")


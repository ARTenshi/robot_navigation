#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

def param_updator():

    pot_fields_d0 = '/obs_detector/pot_fields_d0'
    pot_fields_k_rej = '/obs_detector/pot_fields_k_rej'
    obs_detect_distance = '/obs_detector/max_x'


    if rospy.has_param(pot_fields_d0) and rospy.has_param(pot_fields_k_rej):

        rospy.get_param(pot_fields_d0)
        rospy.get_param(pot_fields_k_rej)
        rospy.get_param(obs_detect_distance)

    else:
        rospy.logwarn(f"Parameter does not exist")

    rospy.logwarn(f"Navigation/PotFields -> Waiting for pot_fields_update msg received!!!")


    while not rospy.is_shutdown():
    
        update_flag = rospy.wait_for_message('/navigation/pot_fields_update', Bool)
    
        if update_flag.data is (True or true):
    
            rospy.logwarn(f"Update Parameter PotFields: d0->0.5, k_rej->2.0")
    
            d0_update = rospy.set_param(pot_fields_d0, 0.5)
            k_rej_update = rospy.set_param(pot_fields_k_rej, 2.0)
            #distance_update = rospy.set_param(obs_detect_distance, 0.5)
            distance_update = rospy.set_param(obs_detect_distance, 0.7)
    
    
        else:
    
            rospy.logwarn(f"Update Parameter PotFields: d0->0.25, k_rej->0.35")
    
            d0_update = rospy.set_param(pot_fields_d0, 0.25)
            k_rej_update = rospy.set_param(pot_fields_k_rej, 0.35)
            distance_update = rospy.set_param(obs_detect_distance, 0.35)
    
    
        
if __name__ == '__main__':

    try:

        rospy.init_node('pot_fields_updator_node')
        pu = param_updator()

    except rospy.ROSInterruptException:
        pass


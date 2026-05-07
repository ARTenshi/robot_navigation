#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty

class HeadPoseCanceller:
    def __init__(self):
        rospy.Subscriber("/navigation/tmp_head_pose_cancel", Empty, self.callback)
        rospy.loginfo("nav_tools->TMPHeadPoseCanceller is ready.")

    def callback(self, msg):
        rospy.logwarn("nav_tools->Temporary move_head_canceller activated!!!")
        rospy.set_param("/simple_move/move_head", False)
        rospy.sleep(2.0)
        rospy.set_param("/simple_move/move_head", True)

if __name__ == "__main__":
    rospy.init_node("head_pose_canceller_topic")
    hpc = HeadPoseCanceller()
    rospy.spin()

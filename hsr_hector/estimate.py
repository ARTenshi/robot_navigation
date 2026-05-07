#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import *
from tf import *

class Initial_pose():
      def __init__(self):
          rospy.init_node("initialize_pose")
          self.pub = rospy.Publisher('laser_2d_correct_pose', PoseWithCovarianceStamped, queue_size = 1)

          self.msg_data = PoseWithCovarianceStamped()
          self.msg_data.header.stamp = rospy.Time.now()
          self.msg_data.header.frame_id = "map"

          self.msg_data.pose.pose.orientation.x =  0.0
          self.msg_data.pose.pose.orientation.y =  0.0
          self.msg_data.pose.pose.orientation.z =  0.0
          self.msg_data.pose.pose.orientation.w = -1.0

          self.msg_data.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853]

          self.main()

      def main(self):
          input_data = raw_input("please input 'x' 'y' 'rad'(each data float type): ")
          p_data = input_data.split(" ")
          self.msg_data.pose.pose.position.x = float(p_data[0])
          self.msg_data.pose.pose.position.y = float(p_data[1])
          self.msg_data.pose.pose.position.z = 0.0
          rospy.loginfo("Setting Pose")
          self.pub.publish(self.msg_data)

if __name__ == "__main__":
    Initial_pose()

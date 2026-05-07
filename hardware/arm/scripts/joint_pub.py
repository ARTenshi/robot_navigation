#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Float32

def publish_torso_height(height):
    pub_lift = rospy.Publisher('/hardware/torso/goal_pose', Float32, queue_size=10)

    lift_msg = Float32()
    lift_msg.data = height

    pub_lift.publish(lift_msg)

def publish_arm_angles(angles):

    pub_arm = rospy.Publisher('/hardware/arm/goal_pose', Float32MultiArray, queue_size=10)

    arm_msg = Float32MultiArray()
    arm_msg.data = angles
    #rospy.loginfo(len(angles))

    pub_arm.publish(arm_msg)



if __name__ == '__main__':
    rospy.init_node('arm_angle_publisher', anonymous=True)
    try:
        # ここでアームの角度を設定します（例として4つのジョイントの角度）
        height = 0.35
        #      arm   flex, roll , gripper flex ,roll
        angles = [-1.57, -1.57, 0.0 ,1.57]  # 角度はラジアンで指定

        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            publish_arm_angles(angles)
            publish_torso_height(height)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass


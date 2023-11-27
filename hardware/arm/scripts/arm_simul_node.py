#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import tf


def callbackPos(msg):
    global goalAngles
    global speeds
    goalAngles = [0, 0, 0, 0]
    speeds = [0.1, 0.1, 0.1, 0.1]
    if len(msg.data) == 4:
        for i in range(len(msg.data)):
            goalAngles[i] = msg.data[i]
    
def main():
    print "INITIALIZING ARM NODE IN SIMULATION BY [EDD-II]"
    ###Connection with ROS
    rospy.init_node("head_simul_node")
    br = tf.TransformBroadcaster()
    jointStates = JointState()
    jointStates.name = ["arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    jointStates.position = [0, 0, 0, 0]

    subPosition = rospy.Subscriber("/hardware/arm/goal_pose", Float32MultiArray, callbackPos)
    pubArmPose = rospy.Publisher("/hardware/arm/current_pose", Float32MultiArray, queue_size = 1)


    #For test, these lines can be unneeded
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    pubArmBattery = rospy.Publisher("/hardware/robot_state/right_arm_battery", Float32, queue_size=1)



    loop = rospy.Rate(10)

    global goalAngles;
    global goalGripper
    global speeds
    goalAngles = [0, 0, 0, 0]
    angles = [0, 0, 0, 0]
    speeds = [0.01, 0.01, 0.01, 0.01]
    goalGripper = 0
    gripper = 0
    gripperSpeed = 0.1

    msgCurrentPose = Float32MultiArray()
    msgCurrentPose.data = [0, 0, 0, 0]
    msgCurrentGripper = Float32()
    msgCurrentGripper.data = 0
    deltaAngles = [0, 0, 0, 0]
    deltaGripper = 0
    while not rospy.is_shutdown():
        for i in range(len(deltaAngles)):
            deltaAngles[i] = goalAngles[i] - angles[i]
            if deltaAngles[i] > speeds[i]:
                deltaAngles[i] = speeds[i]
            if deltaAngles[i] < -speeds[i]:
                deltaAngles[i] = -speeds[i]
            angles[i] += deltaAngles[i]
            jointStates.position[i] = angles[i]
            msgCurrentPose.data[i] = angles[i]

        deltaGripper = goalGripper - gripper
        if deltaGripper > gripperSpeed:
            deltaGripper = gripperSpeed
        if deltaGripper < -gripperSpeed:
            deltaGripper = -gripperSpeed
        gripper += deltaGripper
        
        jointStates.header.stamp = rospy.Time.now()
        pubJointStates.publish(jointStates)
        pubArmPose.publish(msgCurrentPose)
        msgBattery = Float32()
        msgBattery.data = 11.6
        pubArmBattery.publish(msgBattery);
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


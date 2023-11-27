#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import tf

def printHelp():
    print "HEAD NODE. Options:"
    print "TODO: Print all argument options"

def callbackPosHead(msg):
    ### Set GoalPosition
    global goalPan
    global goalTilt
    goalPan = msg.data[0]
    goalTilt = msg.data[1]*-1

def main():
    print "INITIALIZING HEAD NODE IN SIMULATION MODE BY MARCOSOFT..."
    ###Connection with ROS
    rospy.init_node("head")
    br = tf.TransformBroadcaster()
    jointStates = JointState()
    jointStates.name = ["head_pan_joint", "head_tilt_joint"]
    jointStates.position = [0 ,0]

    subPosition = rospy.Subscriber("head/goal_pose", Float32MultiArray, callbackPosHead)
    pubHeadPose = rospy.Publisher("head/current_pose", Float32MultiArray, queue_size = 1);
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    pubHeadBattery = rospy.Publisher("/hardware/robot_state/head_battery", Float32, queue_size=1)
    
    loop = rospy.Rate(30)

    global goalPan
    global goalTilt
    goalPan = 0
    goalTilt = 0
    pan = 0
    tilt = 0
    speedPan = 0.1 #These values should represent the Dynamixel's moving_speed 
    speedTilt = 0.1
    msgCurrentPose = Float32MultiArray()
    msgCurrentPose.data = [0, 0]
    while not rospy.is_shutdown():
        deltaPan = goalPan - pan
        deltaTilt = goalTilt - tilt
        if deltaPan > speedPan:
            deltaPan = speedPan
        if deltaPan < -speedPan:
            deltaPan = -speedPan
        if deltaTilt > speedTilt:
            deltaTilt = speedTilt
        if deltaTilt < -speedTilt:
            deltaTilt = -speedTilt
        pan += deltaPan
        tilt += deltaTilt
        jointStates.header.stamp = rospy.Time.now()
        jointStates.position[0] = pan
        jointStates.position[1] = -tilt #A tilt > 0 goes upwards, but to keep a dextereous system, positive tilt should go downwards
        pubJointStates.publish(jointStates)
        #print "Poses: " + str(panPose) + "   " + str(tiltPose)
        msgCurrentPose.data = [pan, tilt]
        pubHeadPose.publish(msgCurrentPose)
        msgBattery = Float32()
        msgBattery.data = 12.0
        pubHeadBattery.publish(msgBattery)
        loop.sleep()

if __name__ == '__main__':
    try:
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        else:
            main()
    except rospy.ROSInterruptException:
        pass

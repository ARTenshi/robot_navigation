#!/usr/bin/env python3
import copy
from types import resolve_bases
import numpy as np

from std_msgs.msg import Empty
import rospy
import tf
import actionlib

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Float32
from nav_msgs.msg import Path
from actionlib_msgs.msg import GoalStatus
from motion_synth.msg import MotionSynthesisAction, MotionSynthesisResult, MotionSynthesisFeedback

class MotionSynthesisServer:
    def __init__(self):

        self.server = actionlib.SimpleActionServer(
            "/motion_synth",
            MotionSynthesisAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("motion_synth -> Action Server Ready")

        self.arm_pub = rospy.Publisher("/hardware/arm/goal_pose", Float32MultiArray, queue_size=1)
        self.lift_pub = rospy.Publisher("/hardware/torso/goal_pose", Float32, queue_size=1)
        self.head_pub = rospy.Publisher("/hardware/head/goal_pose", Float32MultiArray, queue_size=1)
        #self.tmp_head_pose_canceller = rospy.Publisher("/navigation/tmp_head_pose_cancel", Empty, queue_size=1)

        self.joint_states = {}
        self.joints_cb = rospy.Subscriber("/hsrb/joint_states", JointState, self.joint_state_callback)

        self.current_pose = None
        self.global_pose = rospy.Subscriber("/global_pose", PoseStamped, self.global_pose_callback)

        self.global_nav_goal_reached = False
        self.nav_status_sub = rospy.Subscriber("/navigation/status", GoalStatus, self.nav_status_callback)

    def wait_for_get_path(self):
        rospy.loginfo("motion_synth -> Waiting for get path")
        path_cb = None

        while not rospy.is_shutdown():
            try:
                #msg = rospy.wait_for_message("/simple_move/goal_path", Path, timeout=5.0) #TODO timeout
                msg = rospy.wait_for_message("/simple_move/goal_path", Path) #TODO timeout
                if msg.poses and len(msg.poses) > 0:
                    first_pose = msg.poses[0].pose
                    if first_pose.position.x is not None and first_pose.position.y is not None:
                        path_cb = msg
                        break
            except rospy.ROSException:
                #rospy.logwarn("motion_synth -> Timeout while wait_for_get_path... retrying")
                rospy.logwarn("motion_synth -> Could not get path")
            rospy.sleep(0.1)

        path_points = []
        for pose_stamped in path_cb.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            path_points.append([x, y])

        self.path_len = len(path_points)
        self.path_points = path_points
        rospy.loginfo(f"motion_synth -> Got Path Length: {self.path_len}")

    def nav_status_callback(self, msg):
        text = msg.text
        status = msg.status
        if status==3:
            if text==("Global goal point reached"):
                rospy.loginfo("motion_synth -> Global Nav Goal Reached")
                self.global_nav_goal_reached = True
                

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_states[name] = pos

    def get_joint_positions(self, name):
        return self.joint_states.get(name, float("inf"))

    def joint_goal_reached(self, goal_joints, threshold=0.5):
        diffs = [
            abs(goal_joints.arm_lift_joint - self.get_joint_positions("arm_lift_joint")),
            abs(goal_joints.arm_flex_joint - self.get_joint_positions("arm_flex_joint")),
            abs(goal_joints.arm_roll_joint - self.get_joint_positions("arm_roll_joint")),
            abs(goal_joints.wrist_flex_joint - self.get_joint_positions("wrist_flex_joint")),
            abs(goal_joints.wrist_roll_joint - self.get_joint_positions("wrist_roll_joint")),
            #abs(goal_joints.head_pan_joint - self.get_joint_positions("head_pan_joint")),
            #abs(goal_joints.head_tilt_joint - self.get_joint_positions("head_tilt_joint")),
        ]
        return all(d < threshold for d in diffs)

    def global_pose_callback(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        q = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler =tf.transformations.euler_from_quaternion(q)
        yaw = euler[2]
        self.current_pose = [position.x, position.y, yaw]

    def send_pose(self, joints):

        rospy.loginfo("motion_synth -> Moving Arm State")

        #disable move_head
        #rospy.set_param("/simple_move/move_head", False)

        #TODO disable move_head
        #self.tmp_head_pose_canceller.publish(Empty())

        lift_msg = Float32()
        lift_msg.data = joints.arm_lift_joint

        arm_msg = Float32MultiArray()
        arm_msg.data = [
            joints.arm_flex_joint,
            joints.arm_roll_joint,
            joints.wrist_flex_joint,
            joints.wrist_roll_joint,
        ]

        head_msg = Float32MultiArray()
        head_msg.data = [
            joints.head_pan_joint,
            joints.head_tilt_joint,
        ]

        self.lift_pub.publish(lift_msg)
        self.arm_pub.publish(arm_msg)
        self.head_pub.publish(head_msg)

        #enable move_head
        #rospy.sleep(2.0) #TODO
        #rospy.set_param("/simple_move/move_head", True)

    #TODO add another rule
    def check_self_collision_risk(self, goal_pose):

        if goal_pose.arm_flex_joint < -1.0:
            return True
        
        elif goal_pose.arm_flex_joint > -0.35 and goal_pose.arm_lift_joint > 0.15:
            return True
        
        return False

    def create_temporary_pose(self, goal_pose):
        # if end_pose is in collision
        temporary_pose = copy.deepcopy(goal_pose)

        if goal_pose.arm_flex_joint < -1.0:
            temporary_pose.arm_flex_joint = -0.6 #hsrc
            #temporary_pose.arm_flex_joint = -0.3 #hsrb

        elif goal_pose.arm_flex_joint > -0.35 and goal_pose.arm_lift_joint > 0.15:
            temporary_pose.arm_flex_joint = -0.60
        return temporary_pose

    def execute_cb(self, goal):
        self.global_nav_goal_reached = False

        #both none
        if goal.apply_start_pose is None and goal.apply_goal_pose is None:
            self.send_pose(goal.start_pose)
            rospy.loginfo("motion_synth -> Both Pose has None, Set Default Pose")
            self.server.set_succeeded(MotionSynthesisResult(result=True))
            return

        self.wait_for_get_path()
        feedback = MotionSynthesisFeedback()
        triggered = False
        arm_start_position = int(self.path_len * 0.75)
        rospy.loginfo(f"motion_synth -> motion_start_timing point is {arm_start_position}")

        rate = rospy.Rate(10)

        #start pose only
        if goal.apply_start_pose and not goal.apply_goal_pose:
            self.send_pose(goal.start_pose)
            rospy.loginfo("motion_synth -> Start Pose Only, Skipping Goal Pose")
            self.server.set_succeeded(MotionSynthesisResult(result=True))
            return #important

        #start pose is True or both pose is True
        if goal.apply_start_pose:
            rospy.loginfo("motion_synth -> Start Pose Only, Skipping Goal Pose")
            self.send_pose(goal.start_pose)

        temporary_pose = None
        chk_self_collision = self.check_self_collision_risk(goal.goal_pose)
        if chk_self_collision:
            rospy.logwarn("motion_synth -> Detect Self Collision Pose, Using TMP Arm Pose")
            temporary_pose = self.create_temporary_pose(goal.goal_pose)

        temporary_pose_sent = False
        final_pose_sent = False

        while not rospy.is_shutdown():

            if self.server.is_preempt_requested(): #Preempt
                rospy.logwarn("motion_synth -> Preempt requested, Motion canceled")
                self.server.set_preempted()
                return False

            if self.current_pose and not triggered: #3/4地点到達判定, motion synth start
                cx, cy, _ = self.current_pose
                tx, ty = self.path_points[arm_start_position]
                dist = np.linalg.norm([cx - tx, cy - ty])

                if dist < 1.0:
                    rospy.loginfo("motion_synth -> Reached 3/4 of path & Triggering Motion Synth")
                    if chk_self_collision:
                        self.send_pose(temporary_pose)
                        temporary_pose_sent = True
                    else:
                        self.send_pose(goal.goal_pose)
                        final_pose_sent = True
                    triggered = True

            if triggered:
                if temporary_pose_sent and not final_pose_sent:
                    yaw_error = abs(self.current_pose[2] - goal.goal_location.theta) #yawがgoalに近づいたら最終ポーズを送信
                    if yaw_error > np.pi:
                        yaw_error = 2 * np.pi - yaw_error
                    if yaw_error < 0.3 or self.global_nav_goal_reached: #TODO use angle tolerance from simple_move
                        rospy.loginfo("motion_synth -> Reached Global Nav Goal, sending final pose")
                        self.send_pose(goal.goal_pose)
                        final_pose_sent = True
                        self.global_nav_goal_reached = False

            if triggered and final_pose_sent:
                if self.joint_goal_reached(goal.goal_pose):
                    rospy.loginfo("motion_synth -> Final Arm Pose Reached")
                    self.server.set_succeeded(MotionSynthesisResult(result=True))
                    return

            self.server.publish_feedback(feedback)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("motion_synth_server_for_pumas")
    MotionSynthesisServer()
    rospy.spin()


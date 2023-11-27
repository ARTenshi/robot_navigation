#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped as pwcs
from geometry_msgs.msg import PoseStamped
import tf


global global_pose
global_pose=PoseStamped()

def callback_amcl(msg):
	global_pose.pose=msg.pose.pose

def main():
	global global_pose
	print("Initializing fake_pose_integrator")
	rospy.init_node('fake_pose_integrator', anonymous=True)
	rospy.Subscriber("/navigation/localization/amcl_pose", pwcs, callback_amcl)
	pub_global=rospy.Publisher('/global_pose', PoseStamped, queue_size=1)
	loop=rospy.Rate(100)
	global_pose.header.frame_id="map"
	listener = tf.TransformListener()

	while not rospy.is_shutdown():
		global_pose.header.stamp=rospy.Time.now()
		try:
			(trs, quat)=listener.lookupTransform('map','base_link',  rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		    continue
		global_pose.pose.position.x=trs[0]
		global_pose.pose.position.y=trs[1]
		global_pose.pose.position.z=trs[2]
		global_pose.pose.orientation.x=quat[0]
		global_pose.pose.orientation.y=quat[1]
		global_pose.pose.orientation.z=quat[2]
		global_pose.pose.orientation.w=quat[3]
		pub_global.publish(global_pose)
		loop.sleep()



if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

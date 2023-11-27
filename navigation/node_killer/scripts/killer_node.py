#!/usr/bin/env python
# license removed for brevity
import rospy
import subprocess as sub
import time

def killer():
    rospy.init_node('killer_node', anonymous=True)
    rate = rospy.Rate(3000) # 10hz
    while not rospy.is_shutdown():
        p = sub.Popen(['rosnode', 'ping', '-c', '1', '/pose_integrator'], stdout=sub.PIPE, stderr=sub.PIPE)
        output, errors = p.communicate()

        if(errors):
            continue        
        else:
            p = sub.Popen(['rosnode', 'kill', '/pose_integrator'], stdout=sub.PIPE, stderr=sub.PIPE)
            print("killing: /pose_integrator")

        time.sleep(5)
        rate.sleep()

if __name__ == '__main__':
    try:
        killer()
    except rospy.ROSInterruptException:
        pass

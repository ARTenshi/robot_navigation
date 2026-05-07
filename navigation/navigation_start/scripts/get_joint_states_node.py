#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os

import numpy as np
import rospy
from hsrlib.rosif import ROSInterfaces
from tamlib.node_template import Node


class HSRTeleop(Node):
    def __init__(self) -> None:
        super().__init__()

        self.rosif = ROSInterfaces()
        self.rosif.sub.auto_setup()

        self.names = [
            "arm_lift_joint",
            "arm_flex_joint",
            "arm_roll_joint",
            "wrist_flex_joint",
            "wrist_roll_joint",
            "head_pan_joint",
            "head_tilt_joint",
        ]

    @staticmethod
    def get_index(List, x, default=False) -> int:
        return List.index(x) if x in List else default

    def run(self) -> None:
        if self.run_enable is False:
            return

        states = self.rosif.sub.joint_states()
        for n in self.names:
            idx = self.get_index(states.name, n)
            if "arm_lift_joint" in n:
                print(f"{n}: {round(states.position[idx], 3)}")
            else:
                print(f"{n}: {round(np.rad2deg(states.position[idx]), 2)} [deg]")
        print("--")


def main():
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", 1)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = HSRTeleop()
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            cls.run()
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import rospy
import numpy as np

import roslib
from hsrlib.hsrif import HSRInterfaces
from hsrlib.rosif import ROSInterfaces
hsrif = HSRInterfaces()

sys.path.append(roslib.packages.get_pkg_dir("hma_env_manage") + "/script")
from lib_manage_env import libManageEnv
lme = libManageEnv()


from geometry_msgs.msg import Pose2D
pose = Pose2D(x = 0.0,
              y = 0.0,
              theta = 0.0
              )

#whole_body.move_to_go
init_pose = {"arm_lift_joint" : 0.33,
             "arm_flex_joint" : -0.42,
             "arm_roll_joint" : 0.0,
             "wrist_flex_joint" : -1.14,
             "wrist_roll_joint" : 0.0,
             "head_pan_joint" : 0.0,
             "head_tilt_joint" : 0.14
             }

#def run(self, goal: Union[Pose2D, str], pose = None, destination_name = None, first_rotate = False, nav_type = "pumas", nav_mode = "abs" , nav_timeout = 0.0, goal_distance = None) -> None:

lme.run(goal = pose, pose = init_pose, first_rotate=False, nav_type="pumas", nav_mode = "abs", nav_timeout = 0)




#!/usr/bin/env python

import rospy
import smach
import smach_ros
import tf2_ros
import numpy as np
from navigation_tools.nav_tool_lib import nav_module
import dynamic_reconfigure.client as reconf_client

#from nav_tool_lib import nav_module
#omni_base=nav_module("pumas")

import hsrb_interface

robot = hsrb_interface.Robot()
whole_body = robot.get("whole_body")
# omni_base = robot.get("omni_base") #Standard initialisation (Toyota)
omni_base = nav_module("hsr")  # New initalisation (Pumas)
gripper = robot.get('gripper')
collision = robot.get('global_collision_world')
tf_buffer = robot._get_tf2_buffer()

reconf_base = reconf_client.Client('tmc_map_merger/inputs/base_scan/obstacle_circle')
reconf_head = reconf_client.Client('tmc_map_merger/inputs/head_rgbd_sensor/obstacle_circle')
reconf_depth_obstacle_enable = reconf_client.Client('/tmc_map_merger/inputs/head_rgbd_sensor')
reconf_laser_obstacle_enable = reconf_client.Client('/tmc_map_merger/inputs/base_scan')
##disable xtion obstacle avoidance
reconf_depth_obstacle_enable.update_configuration({"enable":False})


whole_body.move_to_go()
#omni_base.go_rel(1.0, 0, 0, 300)
reconf_laser_obstacle_enable.update_configuration({"enable":False})
omni_base.go_abs(4.65,-0.36,0, 0, "pumas")


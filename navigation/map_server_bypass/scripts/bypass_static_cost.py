#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

class MapMerger:
    def __init__(self):
        rospy.init_node('map_merger')
        
        self.cartographer_map = None
        self.dynamic_obstacle_map = None
        
        rospy.Subscriber('/map', OccupancyGrid, self.cartographer_callback)
        rospy.Subscriber('/dynamic_obstacle_map', OccupancyGrid, self.dynamic_obstacle_callback)
        
        self.merged_map_pub = rospy.Publisher('/augmented_map', OccupancyGrid, queue_size=1)
        
        self.rate = rospy.Rate(1)  # 1 Hz
        
    def cartographer_callback(self, msg):
        self.cartographer_map = msg
        
    def dynamic_obstacle_callback(self, msg):
        self.dynamic_obstacle_map = msg
        
    def world_to_map(self, wx, wy, map_info):
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        resolution = map_info.resolution
        mx = int((wx - origin_x) / resolution)
        my = int((wy - origin_y) / resolution)
        return mx, my

    def merge_maps(self):
        if self.cartographer_map is None or self.dynamic_obstacle_map is None:
            return None
        
        merged_map = OccupancyGrid()
        merged_map.header = self.cartographer_map.header
        merged_map.info = self.cartographer_map.info
        
        cart_data = np.array(self.cartographer_map.data).reshape((self.cartographer_map.info.height, self.cartographer_map.info.width))
        dyn_data = np.array(self.dynamic_obstacle_map.data).reshape((self.dynamic_obstacle_map.info.height, self.dynamic_obstacle_map.info.width))
        
        # dynamic_obstacle_mapの各セルをcartographer_mapの座標系に変換
        for y in range(self.dynamic_obstacle_map.info.height):
            for x in range(self.dynamic_obstacle_map.info.width):
                if dyn_data[y, x] > 0:  # 障害物がある場合のみ処理
                    world_x = x * self.dynamic_obstacle_map.info.resolution + self.dynamic_obstacle_map.info.origin.position.x
                    world_y = y * self.dynamic_obstacle_map.info.resolution + self.dynamic_obstacle_map.info.origin.position.y
                    cart_x, cart_y = self.world_to_map(world_x, world_y, self.cartographer_map.info)
                    
                    if 0 <= cart_x < self.cartographer_map.info.width and 0 <= cart_y < self.cartographer_map.info.height:
                        cart_data[cart_y, cart_x] = max(cart_data[cart_y, cart_x], dyn_data[y, x])
        
        merged_map.data = cart_data.flatten().tolist()
        return merged_map
        
    def run(self):
        while not rospy.is_shutdown():
            merged_map = self.merge_maps()
            if merged_map:
                self.merged_map_pub.publish(merged_map)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        merger = MapMerger()
        merger.run()
    except rospy.ROSInterruptException:
        pass

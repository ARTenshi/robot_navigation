#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.srv import GetMap, GetMapRequest
from geometry_msgs.msg import Point
import numpy as np
#from navigation_tools.

nav = NavModule(select="hsr")
def call_augment_map_service():
    rospy.wait_for_service('/map_augmenter/get_augmented_map')
    try:
        augment_map = rospy.ServiceProxy('/map_augmenter/get_augmented_map', GetMap)
        res = augment_map()
        return res.map
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None

def process_map(map_data):
    # マップの情報を取得
    resolution = map_data.info.resolution
    width = map_data.info.width
    height = map_data.info.height
    origin = map_data.info.origin
    data = np.array(map_data.data).reshape((height, width))
    
    accessible_points = []
    buffer_distance = 0.8  # 障害物からの最小距離 (メートル)
    
    for y in range(height):
        for x in range(width):
            if data[y, x] == 0:  # 空きスペースかどうか
                # セルの座標を計算
                world_x = origin.position.x + x * resolution
                world_y = origin.position.y + y * resolution
                
                # 原点からの距離を計算
                point_distance = np.sqrt(world_x**2 + world_y**2)
                
                # 周囲のセルを確認して障害物からの距離を確認
                in_safe_zone = True
                for dy in range(-1, 2):
                    for dx in range(-1, 2):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            if data[ny, nx] == 100:  # 障害物がある場合
                                neighbor_x = origin.position.x + nx * resolution
                                neighbor_y = origin.position.y + ny * resolution
                                dist_to_obstacle = np.sqrt((world_x - neighbor_x)**2 + (world_y - neighbor_y)**2)
                                if dist_to_obstacle < buffer_distance:
                                    in_safe_zone = False
                                    break
                    if not in_safe_zone:
                        break
                
                if in_safe_zone:
                    rospy.logwarn("Accessible point added: (%f, %f)", world_x, world_y)
                    accessible_points.append((point_distance, Point(world_x, world_y, 0)))
    
    if accessible_points:
        # 原点から最も遠いポイントを選択
        max_distance_point = max(accessible_points, key=lambda p: p[0])[1]
        rospy.loginfo("Farthest Accessible Point: (%f, %f)", max_distance_point.x, max_distance_point.y)
	nav.go_abs(, , -1.57, 0, 'pumas')
	
    else:
        rospy.loginfo("No accessible points found.")

def main():
    rospy.init_node('map_processor')
    
    # マップサービスを呼び出してマップデータを取得
    map_data = call_augment_map_service()
    if map_data:
        process_map(map_data)

    else:
        rospy.loginfo("Failed to retrieve map data.")
    
    rospy.spin()

if __name__ == '__main__':
    main()


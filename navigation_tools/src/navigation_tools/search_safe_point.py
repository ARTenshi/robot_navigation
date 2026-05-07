#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import rospy
import smach
import smach_ros
import numpy as np
from typing import List
from geometry_msgs.msg import Point, Pose, Pose2D, Quaternion

from nav_msgs.msg import OccupancyGrid
from tamlib.node_template import Node


# class SemanticMapServer(Node):
class SearchSafePoint(Node):
    def __init__(self, loglevel='DEBUG') -> None:
        """
        トピック名の初期化など
        """
        try:
            super().__init__()
        except Exception as e:
            self.logwarn("SearchSafePointの2重定義")
            self.logwarn(e)

        self.map_data_msg = OccupancyGrid()
        self.topic_grid_map = "/augment_map"
        self.topic_cost_map = "/dynamic_obstacle_map"
        self.safe_th = 100
        self.debug_mode = False
        self.sub_register("map_data_msg", self.topic_grid_map, queue_size=1, callback_func=self.run)

    def __del__(self):
        return

    def mapdata_to_image(self, map_msg: OccupancyGrid) -> np.array:
        """
        ラスタスキャンされたデータを読み込み、2次元配列に変換する関数
        """
        width, height = map_msg.info.width, map_msg.info.height

        map_data = np.where(map_msg.data == -1, 255, map_msg.data)  # -1（未知）の領域を255に置換する
        map_img = np.array(map_data, dtype=np.uint8).reshape(height, width)

        return map_img

    def get_point_label(self, occupancy_grid_msg: OccupancyGrid, x: float, y: float, input_type="coordinate") -> int:
        """
        マップ座標系で指定されたグリッドのラベルデータを返す関数
        Args:
            occupancy_grid_msg(OccupancyGrid): rosmsgで渡されるマップ情報
            x(float): ラベルデータを取得したい点(x)
            y(float): ラベルデータを取得したい点(y)
            input_type(str): x, yで指定した値の意味
                coordinate: マップ座標系における値
                grid: どのグリッドかを直接指定
        Returns:
            int: 座標に対応するラベル
        """

        # 必要な情報を抽出
        map_data = occupancy_grid_msg.data
        resolution = occupancy_grid_msg.info.resolution
        resolution = 0.05
        width = occupancy_grid_msg.info.width
        map_pose_x = occupancy_grid_msg.info.origin.position.x
        map_pose_y = occupancy_grid_msg.info.origin.position.y

        if input_type == "coordinate":
            # 対象とされたポイントがどのグリッドに対応するのかを求める
            target_map_x = -map_pose_x + x
            target_map_y = -map_pose_y + y

            target_grid_x = int(target_map_x / resolution)
            target_grid_y = int(target_map_y / resolution)
            # ラスタスキャンされているデータに対応する点を求める
            target_pose = (target_grid_y * width) + target_grid_x
        elif input_type == "grid":
            target_pose = (y * width) + x
        else:
            self.logwarn("input_typeは[coordinate]か[grid]かで指定してください")

        # 対象グリッドのラベルデータを取得する
        label_data = map_data[int(target_pose)]
        self.loginfo(label_data)

        return label_data

    def get_area_label(self, occupancy_grid_msg: OccupancyGrid, x: float, y: float, input_type="coordinate", return_size=35, debug=False):
        """
        領域で指定されたエリアのすべてのラベルデータを同一の配列形式で返す関数
        Args:
            occupancy_grid_msg(OccupancyGrid): rosmsgで渡されるマップ情報
            x(float): ラベルデータを取得したい点(x)
            y(float): ラベルデータを取得したい点(y)
            input_type(str): x, yで指定した値の意味
                coordinate: マップ座標系における値
                grid: どのグリッドかを直接指定
            return_size(int):
                指定した点を中心として，どこまでの範囲を取得するか
                （偶数を指定した場合は+1された範囲のデータが返ります）
        Returns:
            np.ndarray: 指定範囲のラベルデータをまとめたもの
        """

        # 必要な情報を抽出
        map_data = occupancy_grid_msg.data
        resolution = occupancy_grid_msg.info.resolution
        width = occupancy_grid_msg.info.width
        height = occupancy_grid_msg.info.height
        map_pose_x = occupancy_grid_msg.info.origin.position.x
        map_pose_y = occupancy_grid_msg.info.origin.position.y

        map_img = np.array(map_data, dtype=np.uint8).reshape(height, width)

        if input_type == "coordinate":
            # 対象とされたポイントがどのグリッドに対応するのかを求める
            target_map_x = -map_pose_x + x
            target_map_y = -map_pose_y + y
            search_area = int(return_size / 2)
            target_grid_x = int(target_map_x / resolution)
            target_grid_y = int(target_map_y / resolution)
            # ラスタスキャンされているデータに対応する点を求める
            # target_pose = (target_grid_y * width) + target_grid_x
        elif input_type == "grid":
            target_grid_x = x
            target_grid_y = y
        else:
            self.logwarn("input_typeは[coordinate]か[grid]かで指定してください")
            return False

        # オーバーフローへの対策
        if target_grid_x - search_area < 0:
            target_grid_x = search_area
        if target_grid_x + search_area > width:
            target_grid_x = width - search_area

        if target_grid_y - search_area < 0:
            target_grid_y = search_area
        if target_grid_y + search_area > height:
            target_grid_y = height - search_area

        # ほしい領域のみを抽出
        trim_map_img = map_img[(target_grid_y - search_area):(target_grid_y + search_area), (target_grid_x - search_area):(target_grid_x + search_area)]
        start_grid = (target_grid_x - search_area, target_grid_y - search_area)

        if debug:
            cv2.imshow("trim_map_img", trim_map_img)
            cv2.waitKey(0)

        return trim_map_img, start_grid

    def get_most_safe_point(self, pose: Pose2D, goal_torelance=7, refer_size=7):
        """
        指定した地点から，近傍にある安全なナビゲーション先を算出する関数
        Args:
            pose: ナビゲーションの目的地
                geometory_msgs.msgのPose2Dで指定
            goal_torelance(int): ナビゲーションの目的地から何グリッド分のずれを許容するか
            refer_size(int): 特定グリッドの危険度を算出する際に，どの範囲までのグリッドを参照するか
        Returns:
            Pose2D
        """
        # グローバルコストマップのトピックをサブスクライブ
        try:
            cost_map_data_msg: OccupancyGrid = rospy.wait_for_message(self.topic_cost_map, OccupancyGrid, timeout=5)
            self.loginfo("receive cost_map data")
        except rospy.exceptions.ROSException as e:
            # サブスクライブ出来なかった場合は入力された値をそのまま返す
            self.logdebug(e)
            self.logwarn("Could not receive cost map.")
            self.logwarn("Return original pose.")
            return pose

        # mapのlinkがはられている座標を取得
        map_pose_x = cost_map_data_msg.info.origin.position.x
        map_pose_y = cost_map_data_msg.info.origin.position.y

        # どの範囲のコストマップ情報を取得するのかを決定する
        return_size = 2 * (goal_torelance + refer_size + 1)
        trim_cost_map, start_grid = self.get_area_label(cost_map_data_msg, pose.x, pose.y, return_size=return_size, debug=self.debug_mode)
        self.logdebug("get selected area's cost map.")

        # 計算範囲の設定と計算用の変数初期化
        range_max = 2 * (goal_torelance + refer_size + 1) - refer_size
        calc_area = int(refer_size / 2)
        safe_grid = (0, 0)
        safe_area_cost = float("inf")
        origin_grid = int(return_size / 2)

        # 目的地のコストが一定以下だった場合，計算を行わない
        refer_cost_map = trim_cost_map[origin_grid:(origin_grid + calc_area), (origin_grid - calc_area):(origin_grid + calc_area)]
        if np.sum(refer_cost_map) < self.safe_th:
            self.loginfo("origin target place is safety")
            return pose

        # TODO: 本来のナビゲーションゴールからなるべく近い点を選ぶように重み付けを行う
        # 計算範囲内のコストの総和を算出し，最も低いものを採用する
        for y in range(int(refer_size / 2), range_max):
            for x in range(int(refer_size / 2), range_max):
                # 計算範囲のコストマップを
                refer_cost_map = trim_cost_map[(y - calc_area):(y + calc_area), (x - calc_area):(x + calc_area)]
                sum_cost_map = np.sum(refer_cost_map)

                ## グリッド単位で見たときの，本来のナビゲーション目的地との距離
                #orig_goal_distance = abs(x - goal_torelance) + abs(y - goal_torelance)

                ## 距離によって，コスト値に重みをつける
                #if orig_goal_distance < 4:
                #    weight = 1.0
                #elif orig_goal_distance < 8:
                #    weight = 1.5
                #elif orig_goal_distance < 16:
                #    weight = 2.0
                #else:
                #    weight = 3.0
                weight = 1.0

                sum_cost_map = weight * sum_cost_map

                # コストが最も低いデータを保存
                if sum_cost_map < safe_area_cost:
                    self.logdebug(sum_cost_map)
                    safe_area_cost = sum_cost_map
                    safe_grid = (x, y)

        # 得られたグリッド情報をマップ座標系に直す
        local_x = (start_grid[0] + safe_grid[0]) * cost_map_data_msg.info.resolution  # マップの原点 + 抽出した領域
        local_y = (start_grid[1] + safe_grid[1]) * cost_map_data_msg.info.resolution

        map_x = local_x + map_pose_x
        map_y = local_y + map_pose_y

        return Pose2D(map_x, map_y, pose.theta)

    def show_cost_map(self):
        map_data_msg = rospy.wait_for_message(self.topic_cost_map, OccupancyGrid)
        self.loginfo("receive map data")
        trim_map_img = self.get_area_label(map_data_msg, 3.7, -0.09, return_size=50, debug=self.debug_mode)

    def run(self, msg):
        self.map_data_msg = msg
        map_img = self.mapdata_to_image(msg)
        # label_data = self.get_point_label(self.map_data_msg, 1.89, -0.642)
        # trim_map_img = self.get_area_label(self.map_data_msg, 3.7, -0.2, return_size=50, debug=self.debug_mode)

if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])
    sms = SearchSafePoint()
    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", 30)

    loop_wait = rospy.Rate(p_loop_rate)
    while not rospy.is_shutdown():
        point = sms.get_most_safe_point(Pose2D(2.1, -0.30, 0))
        print(point.x)
        print(point.y)
        loop_wait.sleep()

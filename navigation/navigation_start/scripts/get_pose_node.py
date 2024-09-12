#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import threading

import dearpygui.dearpygui as dpg
# import pyperclip
import pyclip
import rospy
from hsrlib.utils import description
from tamlib.node_template import Node
from tamlib.tf import Transform, quaternion2euler


class GetPose(Node):
    def __init__(self) -> None:
        super().__init__()

        self.tamtf = Transform()
        self.description = description.load_robot_description()
        self.pos = [0.0, 0.0, 0.0]

    def delete(self) -> None:
        dpg.destroy_context()

    def setup_gui(self) -> None:
        """GUIのセットアップ"""
        dpg.create_context()

        with dpg.font_registry():
            font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
            default_font = dpg.add_font(font_path, 30)
        dpg.bind_font(default_font)

        with dpg.window(label="main", tag="main"):
            dpg.add_text("x   : 0.0", tag="x", pos=[30, 30])
            dpg.add_text("y   : 0.0", tag="y", pos=[30, 70])
            dpg.add_text("yaw : 0.0", tag="yaw", pos=[30, 110])
            dpg.add_button(
                label="Copy", callback=self.copy, pos=[30, 170], width=200, height=100
            )

        thread = threading.Thread(target=self.run)
        thread.start()

        dpg.create_viewport(title="Robot position", width=300, height=300)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("main", True)
        dpg.start_dearpygui()

    def copy(self) -> None:
        """クリップボードに座標をコピー"""
        x, y, yaw = self.pos
        text = f"({x}, {y}, {yaw})"
        # pyperclip.copy(text)
        pyclip.copy(text)

    def run(self) -> None:
        p_loop_rate = rospy.get_param("~loop_rate", 1)
        loop_wait = rospy.Rate(p_loop_rate)

        while not rospy.is_shutdown():
            pose = self.tamtf.get_pose(
                self.description.frame.map, self.description.frame.base
            )
            if pose is None:
                continue
            _, _, yaw = quaternion2euler(pose.orientation)

            self.pos = [
                round(pose.position.x, 3),
                round(pose.position.y, 3),
                round(yaw, 3),
            ]
            dpg.set_value("x", f"x   : {self.pos[0]}")
            dpg.set_value("y", f"y   : {self.pos[1]}")
            dpg.set_value("yaw", f"yaw : {self.pos[2]}")

            self.loginfo(f"x   : {self.pos[0]}")
            self.loginfo(f"y   : {self.pos[1]}")
            self.loginfo(f"yaw : {self.pos[2]}")
            loop_wait.sleep()


def main():
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    cls = GetPose()
    rospy.on_shutdown(cls.delete)
    cls.setup_gui()


if __name__ == "__main__":
    main()

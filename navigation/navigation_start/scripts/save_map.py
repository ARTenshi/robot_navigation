#! /usr/bin/env python3 
# -*- coding: utf-8 -*-
import os
import sys
import shutil
import rospy
import rospkg
import subprocess

class SaveMap:
    def __init__(self):
        rospy.init_node(os.path.basename(__file__).split(".")[0])
        rospack = rospkg.RosPack()
        pkg_dir_path = rospack.get_path("navigation_start")
        self.maps_dir_path = os.path.join(pkg_dir_path, "maps", "maps")
        self.prohibition_maps_dir_path = os.path.join(pkg_dir_path, "maps", "prohibition_maps")
        
        self.p_map_name = rospy.get_param("~map_name", None)
        
        self.run()
    
    def run(self):
        if (self.p_map_name == None):
            print("\033[31m" + "The parameter map_name is not set. _map_name:=<map name>" + "\033[0m")
            sys.exit(1)
        
        if os.path.exists(os.path.join(self.maps_dir_path, self.p_map_name)) or os.path.exists(os.path.join(self.prohibition_maps_dir_path, self.p_map_name)):
            print("\033[31m" + f"Error: Map name '{self.p_map_name}' already exists." + "\033[0m")
            sys.exit(1)
        
        os.makedirs(os.path.join(self.maps_dir_path, self.p_map_name), exist_ok=True)
        result = subprocess.run(["rosrun", "map_server", "map_saver", "-f", os.path.join(self.maps_dir_path, self.p_map_name, 'map')], check=True, text=True)
        if result.stdout is not None and result.stderr is not None:
            print("\033[31m" + "Error stdout: " + result.stdout +  + "\033[0m")
            print("\033[31m" + "Error stderr: " + result.stderr +  + "\033[0m")
        
        if os.path.exists(os.path.join(self.maps_dir_path, self.p_map_name, "map.yaml")) and os.path.exists(os.path.join(self.maps_dir_path, self.p_map_name, "map.pgm")):
            os.makedirs(os.path.join(self.prohibition_maps_dir_path, self.p_map_name), exist_ok=True)
            shutil.copy(os.path.join(self.maps_dir_path, self.p_map_name, "map.yaml"), os.path.join(self.prohibition_maps_dir_path, self.p_map_name))
            shutil.copy(os.path.join(self.maps_dir_path, self.p_map_name, "map.pgm"), os.path.join(self.prohibition_maps_dir_path, self.p_map_name))
        else:
            print("\033[31m" + "Error: map.yaml or map.pgm not found." + "\033[0m")
            sys.exit(1)
        
        if os.path.exists(os.path.join(self.prohibition_maps_dir_path, self.p_map_name, "map.yaml")) and os.path.exists(os.path.join(self.prohibition_maps_dir_path, self.p_map_name, "map.pgm")):
            print("\033[34m" + "Successfully saved the map!!" + "\033[0m")
            sys.exit(0)
        else:
            print("\033[31m" + "Failed to copy map.yaml and map.pgm" + "\033[0m")
            sys.exit(1)

def main():
    save_map = SaveMap()
    save_map.run()

if __name__ == "__main__":
    main()

        
        
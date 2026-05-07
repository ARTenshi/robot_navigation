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
        if self.p_map_name is None:
            print("\033[31m" + "The parameter map_name is not set. _map_name:=<map name>" + "\033[0m")
            sys.exit(1)
        
        map_save_path = os.path.join(self.maps_dir_path, self.p_map_name)
        prohibition_save_path = os.path.join(self.prohibition_maps_dir_path, self.p_map_name)

        if os.path.exists(map_save_path) or os.path.exists(prohibition_save_path):
            print("\033[31m" + f"Error: Map name '{self.p_map_name}' already exists." + "\033[0m")
            sys.exit(1)
        
        os.makedirs(map_save_path, exist_ok=True)

        # Save map using map_saver
        result = subprocess.run(
            ["rosrun", "map_server", "map_saver", "-f", os.path.join(map_save_path, 'map')],
            check=True, text=True, capture_output=True
        )

        if result.stdout:
            print(result.stdout)
        if result.stderr:
            print("\033[31m" + "Error stderr: " + result.stderr + "\033[0m")

        # Replace image's full path with 'map.pgm'
        map_yaml_path = os.path.join(map_save_path, "map.yaml")
        if os.path.exists(map_yaml_path):
            with open(map_yaml_path, "r") as f:
                lines = f.readlines()
            with open(map_yaml_path, "w") as f:
                for line in lines:
                    if line.startswith("image:"):
                        f.write("image: map.pgm\n")
                    else:
                        f.write(line)

        if os.path.exists(map_yaml_path) and os.path.exists(os.path.join(map_save_path, "map.pgm")):
            os.makedirs(prohibition_save_path, exist_ok=True)
            shutil.copy(map_yaml_path, prohibition_save_path)
            shutil.copy(os.path.join(map_save_path, "map.pgm"), prohibition_save_path)
        else:
            print("\033[31m" + "Error: map.yaml or map.pgm not found." + "\033[0m")
            sys.exit(1)
        
        if os.path.exists(os.path.join(prohibition_save_path, "map.yaml")) and os.path.exists(os.path.join(prohibition_save_path, "map.pgm")):
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


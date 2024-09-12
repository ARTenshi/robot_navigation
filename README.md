
# PUMAS Navigation for RoboCup2024 DSPL

# How to usr this repository
## SLAM
docker compose -f slam-docker-compose.yml build
docker compose -f slam-docker-compose.yml up

## LOCALIZATION
docker compose -f localization-docker-compose.yml build
docker compose -f localization-docker-compose.yml up

## MAP BUILDER
following readme at hsr_hector pkg



# robot navigation system introduction
Service robots are intended to help humans in non-industrial environments such as houses or offices. To accomplish their goal, service robots must have several skills such as object recognition and manipulation, face detection and recognition, speech recognition and synthesis, task planning and, one of the most important, navigation in dynamic environments. This repository describes a fully implemented motion-planning system that comprehends from motion and path planning algorithms to spatial representation and behavior-based active navigation.

This paper can be consulted online for free at this [link](https://bit.ly/40YEcZR). The following video shows this system working at RoboCup where we have won the **Smoothest, Safest Navigation Award** in 2022 and 2023.

[![Watch the video](https://img.youtube.com/vi/s2g95Y9Me3c/hqdefault.jpg)](https://www.youtube.com/embed/s2g95Y9Me3c)

Please, if you use this material, don't forget to add the following reference:

```
@article{negrete:2018,
author 		= {Marco Negrete and Jesus Savage and Luis Contreras},
title 		= {{A Motion-Planning System for a Domestic Service Robot}},
journal		= {{SPIIRAS Proceedings}},
volume		= {60},
number		= {5},
pages		= {5--38},
year		= {2018}
}
```

# Setup

1. Create an env of your choice.

e.g.

First, create a workspace:

```
cd ~
mkdir -p nav_ws/src
```

Then, clone this repository into the src folder:

```
cd ~/nav_ws/src
git clone https://github.com/ARTenshi/robot_navigation.git
```


2. Checkout to the correct branch. Available branches are:

* main - raw implementation for navigation considering a command velocity actuator and a laser scan sensor; additionally, an rgbd camera can be used.

* follow_me - raw implementation of a human follower using la laser scan readings to track a person's legs [1].

* hsr_robot - and example implementation of this repository in the HSR robot for the follow me and carry my luggage tasks.

e.g.

```
git checkout hsr_robot
```

3. Build the project:

e.g 

```
cd ~/nav_ws
catkin_make
```

# Robot Navigation

## Structure

**Topics**

TODO

**Services**

TODO

## Initialization

### Prerequisites

To use SLAM and navigation, you need to start a mapping system. 

We have followed the instructions for the *Cartographer ROS for the Toyota HSR* as per this [link](https://google-cartographer-ros-for-the-toyota-hsr.readthedocs.io/en/latest/)

### Follow me (HSR)

Start the navigation:

```
source ~/nav_ws/devel/setup.bash
roslaunch navigation_start navigation.launch
```

Start the task:

```
source ~/nav_ws/devel/setup.bash
roslaunch robot_tasks follow_me.launch
```

### Carry my luggage (HSR)

In this task, the robot helps a person with her/his luggage in an unknown and unconstrained environment and should be able to return to the starting position. We focus on the navigation system here.

Start the cartographer (assuming that the cartographer workspace is *cartographer_ws*):

```
source cartographer_ws/install_isolated/setup.bash
roslaunch cartographer_toyota_hsr hsr_2d.launch
```

Start the navigation with a cartographer (i.e. using a dynamic map):

```
source ~/nav_ws/devel/setup.bash
roslaunch navigation_start navigation_cartographer.launch
```

Start the task:

```
source ~/nav_ws/devel/setup.bash
roslaunch robot_tasks carry_my_luggage.launch
```

### Notes

Relevant parameters can be found in the navigation launch files in ```~/nav_ws/src/robot_navigation/navigation/navigation_start/launch```. In specific: 

**Update your map names**

```
  <arg name="prohibition_map_file"  default="$(find navigation_start)/maps/prohibition_maps/room_512/map.yaml"/>
  <arg name="static_map_file"  default="$(find navigation_start)/maps/maps/room_512/map.yaml"/>
```

For robot localisation, we use the original slam map from the ```map_server map_saver``` in ```maps/maps/```. Additionally, for path planning, we edit the previous map to add prohibited and or closed areas (e.g. considering a laser-scan-based mapping, we add the unmapped table area, or, to travel between two main doors, to avoid surrounding it from the outside, we add closed edges; we do not do this with the localisation map to avoid misslocalisation errors between the real sensor reading and an edited map.)

**Enable/Disable potential fields**

```
<arg name="use_pot_fields" default="True"/>
```
For dynamic obstacle avoidance, we use rejective potential fields that can be enabled or disabled with this parameter. 

Additionally:

```
<param name="pot_fields_k_rej" value="0.4"/>
```
this parameter indicates the obstacle's rejective force importance during navigation (too low means that the robot won't avoid obstacles and too hight means that the robot will avoid obstacles too far from it -- this two extreme cases might cause undesired behaviours).

# References

[1] The human follower implementation is based on this master thesis work:

```
@article{becerra:2012,
author 		= {Marco Becerra-Pedraza and Jesus Savage},
title 		= {{Sistema de seguimiento de personas para un robot movil de servicio}},
journal		= {{UNAM}},
year		= {2012}
}
```

You can download it at the following [link](https://web.siia.unam.mx/siia-publico/v/include/modulo_productos/tesis.php?id=858286) (click on *URL: Ver Tesis*).

# Authors

* **Marco Negrete** - [BioRobotics UNAM](https://biorobotics.fi-p.unam.mx/)
* **Luis Contreras** - [AIBot](http://aibot.jp/)

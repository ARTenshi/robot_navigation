# Augmented gridmaps online for ROS.

For most robot mapping applications ROS utilizes the well known OccupancyGrid.
Simple way to define arbitrary geometry maps with finite precision.  However
this format is really akward if you want or need to use for something else.
Specifically dynamic updates to the map.  That is by desing, ros map_server was
created to provide a well stablished static map to other nodes. Not to modify
it at run-time. In occassions it would be helpful to modify occupancy grids for
the long run, for example if an unmapped obstacle has appeared on run-time. 

This small package was created to solve that issue, it allows to add, or
augment the map with obstacles on runtime and republished this enhanced map to
the rest of subscribed nodes. This can be helpfull when these new obstacles are
not gonna leave the robot workspace and you want you planner to avoid trying to
get through them.

## Dependencies

This package was created with simplicity in mind. It only depends on core ROS
libraries and messages so as long as you have a working ROS install it should
work.  This should also be quite compatible between ros versions, but was only
tested on ros-melodic.

## Installation

This should be pretty simple. Download repo to your catking workspace and run
either catkin or catkin_make. As long as you have a base ROS install it should
compile.

##  Usage

This package only has one node: augmented_gridmaps_node. It reads from `/map`
and publishes to `/augmented_map`.

For now it only receives point-type obstacles or a given radius. These need to
be published to `/point_obstacle` topic as PointStamped geometry msgs.

To run a simple demo you can use the provide launch file:

```bash

 roslaunch augment_gridmap_online augment_gridmap_online.launch 

``` 

Thi is requires ros map server to run. From there you can use the `Publish point`
button on rviz to dynamically add obstacles to the map. 

## Nodes

### augment_gridmap_online_node

Single entry point for the package, setups the enhanced map  which 
contains the original map plus added obstacles.

#### Subscribed Topics

- map (nav_msgs::OccupancyGrid) 

    Original map to be augmented dynamically

- point_obstacle (geometry_msgs::PointStamped)

    Obstacle location as a point, radius is determined via parameters.  Code
internally checks it is on same frame as map but won't convert it itself

#### Published Topics 

- augmented_map (nav_msgs::OccupancyGrid)

    Map with augmented obstacles, latched.

- augmented_map_metadata (nav_msgs::MapMetadata)

    Header info for map, for now it is the same as the original map
    But with different timestamp

- obstacle_markers (visualization_msgs::Marker)
  
    Visualization messages this displays obstacles as a small sphere
    on RVIZ

#### Services

- clear_map (std_srvs::Empty)
    
    Removes all markers and osbtacles from the map and returns it 
    to original state

#### Parameters

- ~radius (float, default: 0.05)

    Radius size for obstacle in meters                     




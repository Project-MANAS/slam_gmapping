# SLAM_GMAPPING

SLAM(Simultaneous Localization and Mapping) is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.

This contains package ``openslam_gmapping`` and ``slam_gmapping`` which is a ROS2 wrapper for OpenSlam's Gmapping. Using slam_gmapping, you can create a 2-D occupancy grid map (like a building floorplan) from laser and pose data collected by a mobile robot.

## Launch:

```bash
ros2 launch slam_gmapping slam_gmapping.launch.py
```

The node slam_gmapping subscribes to sensor_msgs/LaserScan on ros2 topic ``scan``. It also expects appropriate TF to be available.

It publishes the nav_msgs/OccupancyGrid on ``map``.

Map Meta Data and Entropy is published on ``map_metadata`` and ``entropy`` respectively.


## Test with turtlebot3:

#### Install Nav2 packages

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-turtlebot3*
```



#### Launch

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

```
ros2 launch slam_gmapping slam_gmapping.launch.py

ros2 run rviz2 rviz2 -d ~/slam_gmapping/gmapping.rviz
```

you can use explore lite ore teleop

```
ros2 run turtlebot3_teleop teleop_keyboard
```
![dynamic transform](slam_gmapping/media/gmapping_test.gif)

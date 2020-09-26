# point cloud storer

![CI](https://github.com/amslabtech/point_cloud_storer/workflows/CI/badge.svg)

## Enviornment
- Ubuntu 18.04 or 20.04
- ROS Melodic or Noetic

## Requirement
- PCL 1.8

## Install and Build
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/amslabtech/point_cloud_storer.git
$ cd ~/catkin_ws
$ catkin_make
```

## Nodes
### point_cloud_storer
#### Published topics
- /cloud/stored (sensor_msgs/PointCloud2)

#### Subscribed topics
- /cloud (sensor_msgs/PointCloud2)
- /odom (nav_msgs/Odometry)

#### params
- TARGET_FRAME
- SOTRE_NUM


### point_cloud_storer
#### Published topics
- /cloud/transformed (sensor_msgs/PointCloud2)

#### Subscribed topics
- /cloud (sensor_msgs/PointCloud2)

#### params
- TARGET_FRAME

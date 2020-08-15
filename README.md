# roverpi_ros

# Dependencies

- rplidar
- laser_scan_matcher
- openslam_gmapping
- slam_gmapping
- hector_slam

# Instruction

**SLAM using gmapping without 'odom'**

Use 'laser_scan_matcher' to get estimated position.

Bring up robot and start gmapping in Gopigo3 robot side:

```
ssh pi@192.168.0.xxx
roslaunch roverpi_ros roverpi_slam_gmapping.launch
```

Launch rviz in Remote PC or Laptop:

```
roslaunch roverpi_ros roverpi_slam_rviz.launch
```

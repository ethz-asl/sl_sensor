# sl_sensor
Code package that runs with the SL Sensor

## Required Packages

* Specific catkin packages
  * versavis (devel/sl_sensor branch from [forked repo](https://github.com/tengfoonglam/versavis/tree/devel/sl_sensor))
  * flir_camera_driver (devel/sl_sensor branch from [forked repo](https://github.com/tengfoonglam/flir_camera_driver)) 
  * [libpointmatcher](https://github.com/ethz-asl/libpointmatcher)
  * [libpointmatcher_ros](https://github.com/ethz-asl/ethzasl_icp_mapping)
  * [eigen_catkin](https://github.com/ethz-asl/eigen_catkin)
* Standard ROS packages
  * roscpp
  * geometry_msgs
  * sensor_msgs
  * pcl_ros
  * cv_bridge
  * tf_conversions
* External Libraries
  * [yaml-cpp](https://github.com/jbeder/yaml-cpp)
  * OpenCV 3.2.0 with extra modules (opencv_contrib)
  * Point Cloud Library (PCL) Version 1.8


## Running the Example Code

There are two examples that are provided. 

1. **pairwise_registration_demo** shows code on how to merge multiple point clouds using pairwise registration
2. **structured_light_odometry** runs the VO pipeline for the structured light sensor with the rumlang1 rosbag

Steps to run examples:
1. Compile the sl_sensor package
2. Download and extract the *ma_thesis_submission_dataset*
3. To run **pairwise_registration_demo**:
   1. Open ./core/sl_sensor_registration/launch/pairwise_registration_demo.launch
   2. Edit the directories in the launch file to point to the extracted folder
   3. Run the command `roslaunch sl_sensor_registration pairwise_registration_demo.launch`
4. To run **structured_light_odometry**:
   1. Open ./core/sl_sensor_vo/launch/structured_light_odometry.launch
   2. Edit the directories in the launch file to point to the extracted folder
   3. Run the command `roslaunch sl_sensor_vo structured_light_odometry.launch`

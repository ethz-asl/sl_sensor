# sl_sensor

SL Sensor is an open-source, ROS-based, structured light sensor for high-accuracy 3D scanning. It can produce high-fidelity point clouds in real-time at 5Hz using hardware triggering of the camera and the projector.

## System Requirements

* Ubuntu 18.04 with ROS Melodic installed
* Required Libraries (* included in 18.04 installation)
  *   *yaml-cpp (Version 0.5.2)
  *   *OpenCV 3.2.0
  *   *Point Cloud LIbrary PCL (Version 1.8)
  *   [Spinnaker](https://flir.app.boxcn.net/v/SpinnakerSDK/folder/68522911814) (Version 2.4.0.143) 
  *   [Open3d](https://github.com/isl-org/Open3D) (Version 0.13.0) NOTE: Please refer to this [issue](https://github.com/ros-perception/perception_open3d/issues/16) to ensure that you can build and install Open3d so that it is ROS-compatible

## Contained Packages

* sl_sensor_calibration: Calibration utilities for the SL Sensor
* sl_sensor_codec: Generate different structured light patterns (encoding) and processing images to obtain projector coordinates (decoding)
* sl_sensor_image_acquisition: Group images that are from the same pattern sequence together from timestamped image streams from Versavis 
* sl_sensor_logger: Logging utilities to save images and point clouds
* sl_sensor_motion_compensation: Motion compensation algorithm that enables scanning while in motion (constrained to linear motion at the moment)
* sl_sensor_registration: Basic utilities to register point cloud in a pairwise fashion using ICP
* sl_sensor_rqt_gui: Simple GUI for calibration and scanning
* sl_sensor_timer: Basic timing utilities

## Build

1. Set up your Personal Access Token so you can clone over SSH
2. Install ros-melodic
3. Clone this repository into the src folder of your workspace.
4. Configure your workspace to build in release: catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
5. Install vcstool and catkin-tools from apt: sudo apt install python3-catkin-tools python3-vcstool
6. From the src folder of your workspace, clone source dependencies with vcs tool: vcs import < sl_sensor/dependencies.vcs
7. From the src folder of your workspace, install the system dependencies with rosdep: rosdep install --from-paths . --ignore-src -r -y
8. Build the FLIR camera driver 'catkin build flir_camera_driver'
9. Build the SL Sensor package 'catkin build sl_sensor'

## Running the Example Code

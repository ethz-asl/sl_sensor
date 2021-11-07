<img src="https://user-images.githubusercontent.com/19413243/140636195-c58c5ec1-e38b-4b75-a1fe-b03624708d1f.png" alt="drawing" width="250"/>

#

SL Sensor is an open-source, ROS-based, structured light sensor for high-accuracy 3D scanning. It can produce high-fidelity point clouds in real-time at 5Hz using hardware triggering of the camera and the projector.

<img src="https://user-images.githubusercontent.com/19413243/134910454-87785a6d-0c3f-4dab-95fc-e076042a359c.png" alt="drawing" width="500"/>

Refer to the Wiki pages for the wiring schematics, CAD as well as steps to calibrate/setup the sensor.

The sample build we have provided contains two cameras (because we want it as a research platform to test our linear motion compensation strategy in both directions) but it can be modified to work with only one camera as well.

Some experience with C++ and ROS is required to troubleshoot and configure roslaunch files to make it work on your system.

## Videos

Work in progress

## System Requirements

* Computer/Laptop with at least one USB3 Port
* Ubuntu 18.04 with ROS Melodic installed
* Required Libraries (* included in 18.04 installation)
  *   *yaml-cpp (Version 0.5.2)
  *   *OpenCV 3.2.0
  *   *Point Cloud Library PCL (Version 1.8)
  *   [Spinnaker](https://flir.app.boxcn.net/v/SpinnakerSDK/folder/68522911814) (Version 2.4.0.143) 
  *   [Open3d](https://github.com/isl-org/Open3D) (Version 0.13.0) NOTE: Please refer to this [issue](https://github.com/ros-perception/perception_open3d/issues/16) to ensure that you can build and install Open3d so that it is ROS-compatible

## Contained Packages

* sl_sensor_calibration: Calibration utilities for the SL Sensor
* sl_sensor_codec: Generate different structured light patterns (encoding) and processing images to obtain projector coordinates (decoding)
* sl_sensor_image_acquisition: Group images that are from the same pattern sequence together from timestamped image streams from Versavis 
* sl_sensor_logger: Logging utilities to save images and point clouds
* sl_sensor_motion_compensation: Motion compensation algorithm that enables scanning while in motion (constrained to linear motion at the moment)
* sl_sensor_reconstruction: Converts decoded projector coordinate images into point clouds
* sl_sensor_registration: Basic utilities to register point cloud in a pairwise fashion using ICP
* sl_sensor_rqt_gui: Simple GUI for calibration and scanning
* sl_sensor_timer: Basic timing utilities

## Build

1. Set up your Personal Access Token so you can clone over SSH
2. Install ros-melodic
3. Clone this repository into the src folder of your workspace
4. Configure your workspace to build in release: catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
5. Install vcstool and catkin-tools from apt: sudo apt install python3-catkin-tools python3-vcstool
6. From the src folder of your workspace, clone source dependencies with vcs tool: vcs import < sl_sensor/dependencies.vcs
7. From the src folder of your workspace, install the system dependencies with rosdep: rosdep install --from-paths . --ignore-src -r -y
8. Build the FLIR camera driver `catkin build flir_camera_driver`
9. Build the SL Sensor package `catkin build sl_sensor`

## Making sure that the package is working

1. Download the parameter files and rosbags from [here](https://drive.google.com/drive/folders/15cGJHTtFs545hII7yK-zT51W3buLjmKb?usp=sharing)
2. Replace core/sl_sensor_codec/codec_yaml/codec_config.yaml with the codec_config.yaml in the downloaded folder parameter_files
3. Replace the xml files primary_camera.xml, secondary_camera.xml, projector.xml core/sl_sensor_calirbation/calibration_files with those in the downloaded folder parameter_files
4. Start a roscore in a terminal by running `roscore`
5. On a separate terminal run `rosparam set /use_sim_time true`
6. Roslaunch the reconstruction pipeline that has the same pattern name as the rosbag in the downloaded folder
   * For example, if you want to run `rosbag play --clock 2p1_tpu_horizontal.bag` (make sure terminal is in the 2p1_tpu_horizontal folder)
   * Run `roslaunch sl_sensor 2p1_tpu_horizontal_motion_compensation.launch` to see motion compensation in action
   * Run `roslaunch sl_sensor 2p1_tpu_horizontal.launch` if you want to see the distortion due to motion if a standard reconstruction pipeline is used

## Acknowledgements
This repo contains code adapted from:
 * Reconstruction pipeline components - [SLStudio](https://github.com/jakobwilm/slstudio)
 * Bundle Adjustment dual camera-projector calibration (experimental) [IDIAP Multicam Calibration](https://github.com/idiap/multicamera-calibration) 

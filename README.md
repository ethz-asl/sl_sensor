<img src="https://user-images.githubusercontent.com/19413243/140749627-b45cf6cd-61ed-4880-b4b3-86a8684c82fe.png" alt="drawing" width="250"/>

#

**SL Sensor** is an open-source, ROS-based, structured light sensor for high-accuracy 3D scanning. It can produce high-fidelity point clouds in real-time at 5Hz using hardware triggering of the camera and the projector. Detailed information can be found in this [publication](https://www.sciencedirect.com/science/article/pii/S0926580522002977).

<img src="https://user-images.githubusercontent.com/19413243/134910454-87785a6d-0c3f-4dab-95fc-e076042a359c.png" alt="drawing" width="500"/>

Refer to the [Wiki pages](https://github.com/ethz-asl/sl_sensor/wiki) for the wiring schematics, CAD as well as steps to calibrate/setup the sensor.

The sample build we have provided contains two cameras (because we want it as a research platform to test our linear motion compensation strategy in both directions) but it can be modified to work with only one camera as well.

Some experience with C++ and ROS is required to troubleshoot and configure roslaunch files to make it work on your system.

## Demos

### Scanning of Brick and Mortar Wall

https://user-images.githubusercontent.com/19413243/140678239-58d45401-ebac-4b78-907e-b910440c43c0.mov

### Scans of Table Tennis Balls (vs RealSense L515)

<img src="https://user-images.githubusercontent.com/19413243/140750462-cd4e6b25-6e07-4b35-bfba-e862faf8493b.png" alt="drawing" width="1000"/>

### Linear Motion Compensation

https://user-images.githubusercontent.com/19413243/140685541-cc4db604-444c-4e43-932a-87a7f24c87e4.mov

### Colour Scanning (Depth Information using top camera, RGB information using left camera)

https://user-images.githubusercontent.com/19413243/140679279-821adcbb-1930-419e-af2e-e022aaa04d4a.mov

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
10. To ensure the scannning RQt dashboard loads properly, change the directories in the following RQt perspective files to reflect the actual location in your computer
    * sl_sensor/core/sl_sensor_rqt_guis/sl_sensor_rqt_guis_common/rqt_perspectives/scan_gui_monochrome.perspective: <br />Change line 527 ("repr": "u'/home/ltf/catkin_ws/src/sl_sensor/sl_sensor/rviz/scan_rqt_gui_monochrome.rviz'")
    * sl_sensor/core/sl_sensor_rqt_guis/sl_sensor_rqt_guis_common/rqt_perspectives/scan_gui_colour.perspective: <br />Change line 527 ("repr": "u'/home/ltf/catkin_ws/src/sl_sensor/sl_sensor/rviz/scan_rqt_gui_colour.rviz'")

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

## Citing this Work
If you used this repository for academic research, please cite the following publication:

```
@article{TengFoongLamSLSensor2022,
title = {SL Sensor: An open-source, real-time and robot operating system-based structured light sensor for high accuracy construction robotic applications},
journal = {Automation in Construction},
volume = {142},
pages = {104424},
year = {2022},
issn = {0926-5805},
doi = {https://doi.org/10.1016/j.autcon.2022.104424},
url = {https://www.sciencedirect.com/science/article/pii/S0926580522002977},
author = {Teng Foong Lam and Hermann Blum and Roland Siegwart and Abel Gawel}
}
```

## Acknowledgements
This repo contains code adapted from:
 * Reconstruction pipeline components - [SLStudio](https://github.com/jakobwilm/slstudio)
 * Bundle Adjustment dual camera-projector calibration (experimental) [IDIAP Multicam Calibration](https://github.com/idiap/multicamera-calibration) 

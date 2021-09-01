#!/bin/bash

roslaunch sl_sensor_calibration prepare_bundle_adjustment.launch;
roslaunch sl_sensor_calibration run_bundle_adjustment.launch;


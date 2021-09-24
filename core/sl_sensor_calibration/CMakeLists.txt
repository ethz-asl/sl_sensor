cmake_minimum_required(VERSION 2.8.3)
project(sl_sensor_calibration)
find_package(OpenCV REQUIRED)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-Wall -Wextra)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -lstdc++fs")

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/calibration_sequence_acquisition_nodelet.cpp
  src/calibrator.cpp 
  src/intrinsic_parameters.cpp
  src/camera_parameters.cpp
  src/dual_camera_calibration_preparator.cpp
  src/calibration_utilities.cpp
  src/BAProblem.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC "/include" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} stdc++fs)

##############
# EXECUTABLES #
##############

cs_add_executable(calibrator_node src/calibrator_node.cpp)
target_link_libraries(calibrator_node ${PROJECT_NAME} ${OpenCV_LIBS} stdc++fs yaml-cpp)

cs_add_executable(dual_camera_calibration_preparator_node src/dual_camera_calibration_preparator_node.cpp)
target_link_libraries(dual_camera_calibration_preparator_node ${PROJECT_NAME} ${OpenCV_LIBS} stdc++fs yaml-cpp)

cs_add_executable(run_bundle_adjustment_node src/run_bundle_adjustment_node.cpp)
target_link_libraries(run_bundle_adjustment_node ${PROJECT_NAME} ${OpenCV_LIBS})

cs_install_scripts(python_scripts/visualise_residuals.py)

##########
# EXPORT #
##########

cs_install()
cs_export()

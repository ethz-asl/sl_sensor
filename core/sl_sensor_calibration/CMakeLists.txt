cmake_minimum_required(VERSION 2.8.3)
project(sl_sensor_calibration)
find_package(OpenCV REQUIRED)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/calibration_data.cpp
  )


target_include_directories(${PROJECT_NAME} PUBLIC "/include")
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

##########
# EXPORT #
##########
cs_install()
cs_export()

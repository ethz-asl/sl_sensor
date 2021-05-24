cmake_minimum_required(VERSION 2.8.3)
project(sl_sensor_frame_grabber)
find_package(OpenCV REQUIRED)

find_package(catkin_simple REQUIRED)


catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/frame_grabber_nodelet.cpp
  src/frame_logger_nodelet.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC "/include")
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

##########
# EXPORT #
##########
cs_install()
cs_export()

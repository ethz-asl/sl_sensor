cmake_minimum_required(VERSION 2.8.3)
project(sl_sensor_vo)

find_package(PCL REQUIRED)
find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17 -Wall -Wextra -fPIC)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/visual_odometry_frame.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC "/include" ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} )

##########
# EXPORT #
##########
cs_install()
cs_export()

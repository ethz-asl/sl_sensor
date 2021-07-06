cmake_minimum_required(VERSION 2.8.3)
project(sl_sensor_motion_compensation)
find_package(OpenCV REQUIRED)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-Wall -Wextra)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-lstdc++fs" )

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/phase_correlation_utils.cpp
  src/linear_motion_compensation_nodelet.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC "/include")
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

##########
# EXPORT #
##########

cs_install()
cs_export()

cmake_minimum_required(VERSION 2.8.3)
project(sl_sensor_vo)

find_package(PCL REQUIRED)
find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-Wall -Wextra -fPIC)
set (CMAKE_CXX_STANDARD 17)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/visual_odometry_frame.cpp
        src/structured_light_odometry.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC "/include" ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)


##############
# EXECUTABLES #
##############
cs_add_executable(structured_light_odometry_node src/structured_light_odometry_node.cpp)
target_link_libraries( structured_light_odometry_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()

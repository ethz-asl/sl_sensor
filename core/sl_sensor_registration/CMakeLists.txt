cmake_minimum_required(VERSION 2.8.3)
project(sl_sensor_registration)

find_package(PCL REQUIRED)
find_package(Open3D REQUIRED)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-Wall -Wextra -fPIC)
set (CMAKE_CXX_STANDARD 17)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/point_cloud_registration_algorithm.cpp
	src/libpointmatcher_icp.cpp
  src/o3d_color_icp.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC "/include" ${PCL_INCLUDE_DIRS} ${Open3D_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} Open3D::Open3D)

##############
# EXECUTABLES #
##############

cs_add_executable(pairwise_registration_node src/pairwise_registration_node.cpp)
target_link_libraries(pairwise_registration_node ${PROJECT_NAME})

cs_add_executable(pairwise_registration_rosbag_node src/pairwise_registration_rosbag_node.cpp)
target_link_libraries(pairwise_registration_rosbag_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()

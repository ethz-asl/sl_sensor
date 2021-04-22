cmake_minimum_required(VERSION 2.8.3)
project(sl_sensor_visualise)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17 -Wall -Wextra -fPIC)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/pose_stamped_to_path.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC "/include")
target_link_libraries(${PROJECT_NAME})

##############
# EXECUTABLES #
##############

cs_add_executable(pose_stamped_to_path_node src/pose_stamped_to_path_node.cpp)
target_link_libraries(pose_stamped_to_path_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()

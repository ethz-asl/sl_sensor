cmake_minimum_required(VERSION 2.8.3)
project(sl_sensor_visualise)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-Wall -Wextra -fPIC)
set (CMAKE_CXX_STANDARD 17)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/pose_stamped_to_path.cpp
        src/show_image_array_nodelet.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC "/include")
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

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

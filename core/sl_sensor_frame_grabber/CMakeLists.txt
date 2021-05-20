cmake_minimum_required(VERSION 2.8.3)
project(sl_sensor_frame_grabber)

find_package(catkin_simple REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17 -Wall -Wextra)

add_subdirectory(third_party/lc4500api)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  )

target_include_directories(${PROJECT_NAME} PUBLIC "/include" ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} lc4500api yaml-cpp)

add_executable(lightcrafter_4500_node src/lightcrafter_4500_node.cpp)
target_link_libraries(lightcrafter_4500_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()

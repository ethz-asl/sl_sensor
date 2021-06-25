cmake_minimum_required(VERSION 2.8.3)
project(sl_sensor_image_acquisition)
find_package(OpenCV REQUIRED)

find_package(catkin_simple REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-Wall -Wextra)
set (CMAKE_CXX_STANDARD 17)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/image_grouper.cpp
  src/image_array_logger_nodelet.cpp
  src/image_synchroniser_nodelet.cpp
  src/image_array_utilities.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC "/include" ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} yaml-cpp ${OpenCV_LIBS})

##########
# EXPORT #
##########
cs_install()
cs_export()
